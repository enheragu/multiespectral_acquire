/**
 * @file    ouster_sync_node.cpp
 * @brief   C++ replacement for the 5 Python Ouster `_sync` buffer handlers in
 *          buffer_compositor_node.py (NORMAL mode: buffer + republish, no disk).
 *
 * Replaces:
 *   buffer_range_sync   /ouster/range_image  -> ouster/range_image_sync
 *   buffer_reflec_sync  /ouster/reflec_image -> ouster/reflec_image_sync
 *   buffer_signal_sync  /ouster/signal_image -> ouster/signal_image_sync
 *   buffer_nearir_sync  /ouster/nearir_image -> ouster/nearir_image_sync
 *   buffer_pointcloud_sync /ouster/points    -> ouster/points_sync
 *
 * Logic cloned EXACTLY from buffer_handler_node.py (the GenericBufferHandler) for
 * the 5 Ouster handlers in normal mode (store_data=False):
 *
 *  - Buffer each incoming Ouster message keyed by:
 *        ts = header.stamp + clock_offset_ns + fov_offset_ns
 *    where clock_offset_ns comes from /ouster/clock_offset (Float64 SECONDS,
 *    truncated to int ns) and fov_offset_ns is the SHARED lidar scan offset
 *    (computed only by the pointcloud handler, read by the 4 image handlers).
 *
 *  - On each trigger (ImageWithMetadata):
 *        sync_timestamp = metadata.header.stamp(ns) + metadata.exposure_time/2
 *        base_name      = metadata.img_name
 *    Each of the 5 streams runs find_closest(sync_timestamp, 0.06s). If no match
 *    AND the buffer's closest is NOT newer than the trigger, the trigger is
 *    enqueued as pending and re-matched on every subsequent incoming Ouster
 *    frame (the cloud arrives ~50 ms AFTER the trigger — pending-retry is
 *    ESSENTIAL or the yield collapses).
 *
 *  - On match: scan-offset is refreshed (pointcloud only, BEFORE restamp), the
 *    matched message's header.stamp is rewritten to (orig + clock_offset + fov),
 *    header.frame_id is set to base_name, and it is republished on the sync topic.
 *
 * Deviations from Python are documented inline and in the StructuredOutput notes
 * (mainly: fail-loud if no clock_offset within 5 s of first trigger; configurable
 * output suffix for parallel testing).
 */
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"

#include "multiespectral_acquire/msg/image_with_metadata.hpp"

using std::placeholders::_1;

namespace {

inline int64_t stampToNs(const builtin_interfaces::msg::Time & t) {
  return static_cast<int64_t>(t.sec) * 1000000000LL + static_cast<int64_t>(t.nanosec);
}

inline void nsToStamp(int64_t ns, builtin_interfaces::msg::Time & t) {
  // Python: sec = ns // 1e9 ; nanosec = ns % 1e9 (floored division, nonneg remainder).
  int64_t sec = ns / 1000000000LL;
  int64_t rem = ns % 1000000000LL;
  if (rem < 0) { rem += 1000000000LL; sec -= 1; }
  t.sec = static_cast<int32_t>(sec);
  t.nanosec = static_cast<uint32_t>(rem);
}

}  // namespace

/**
 * One BufferHandler per Ouster stream. Templated on the message type
 * (sensor_msgs::msg::Image for the 4 images, PointCloud2 for the cloud).
 *
 * Holds its own ring buffer + pending-trigger queue, both mutex-guarded so the
 * data callback (writer thread) and the trigger callback (trigger thread) can run
 * concurrently under a MultiThreadedExecutor.
 */
template <typename MsgT>
class BufferHandler {
public:
  using MsgConstPtr = typename MsgT::ConstSharedPtr;
  // A buffered entry: corrected timestamp (ns) + an OWNED copy of the message so
  // we can restamp it independently when it matches (Python mutates msg in place;
  // we copy so concurrent republishes of the same buffered frame stay correct).
  struct Entry {
    int64_t timestamp_ns;
    std::shared_ptr<MsgT> msg;
  };
  struct Pending {
    int64_t sync_timestamp;
    std::string base_name;
  };

  BufferHandler(rclcpp::Node * node,
                const std::string & data_topic,
                const std::string & sync_topic,
                size_t buffer_cap,
                int64_t max_time_diff_ns,
                bool is_pointcloud,
                const std::atomic<int64_t> & shared_scan_offset_ns,
                std::atomic<int64_t> & shared_scan_offset_writable,
                const std::atomic<int64_t> & clock_offset_ns,
                bool calibration_mode,
                int64_t store_min_interval_ns,
                const std::atomic<bool> & recording_enabled)
  : node_(node),
    buffer_cap_(buffer_cap),
    max_time_diff_ns_(max_time_diff_ns),
    is_pointcloud_(is_pointcloud),
    shared_scan_offset_ns_(shared_scan_offset_ns),
    shared_scan_offset_writable_(shared_scan_offset_writable),
    clock_offset_ns_(clock_offset_ns),
    calibration_mode_(calibration_mode),
    store_min_interval_ns_(store_min_interval_ns),
    recording_enabled_(recording_enabled) {
    // Data sub: RELIABLE, KEEP_LAST, depth = buffer_cap (>=40). C++ is fast and
    // will not back-pressure; matches the compositor's RELIABLE Ouster intake.
    auto data_qos = rclcpp::QoS(rclcpp::KeepLast(buffer_cap_)).reliable();
    sub_ = node_->create_subscription<MsgT>(
        data_topic, data_qos,
        std::bind(&BufferHandler::dataCb, this, _1));
    // Sync pub: depth 10, RELIABLE (default). The large _sync clouds/images MUST
    // be reliable (a best-effort reader silently drops ~70% of the big samples).
    pub_ = node_->create_publisher<MsgT>(sync_topic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    RCLCPP_INFO(node_->get_logger(), "[ouster_sync] %s -> %s (cap %zu, %s)",
                data_topic.c_str(), sync_topic.c_str(), buffer_cap_,
                is_pointcloud_ ? "pointcloud" : "image");
  }

  // ---- incoming Ouster data ----
  void dataCb(MsgConstPtr msg) {
    const int64_t raw_ns = stampToNs(msg->header.stamp);
    // _fov_offset_ns: the SHARED scan offset (0 until the first matched cloud).
    const int64_t ts = raw_ns + clock_offset_ns_.load() + shared_scan_offset_ns_.load();
    {
      std::lock_guard<std::mutex> lk(buf_mtx_);
      buffer_.push_back(Entry{ts, std::make_shared<MsgT>(*msg)});
      while (buffer_.size() > buffer_cap_) buffer_.pop_front();
    }
    // handler:511 — every data arrival re-runs all pending sync requests.
    retryPending();
  }

  // ---- incoming trigger ----
  // sync_timestamp + base_name already computed by the node (shared across all 5).
  //
  // BUG 1 / DEVIATION 3 fix: all match-and-publish for THIS stream is serialized
  // under match_mtx_, so onTrigger (trigger thread, direct findClosest) and
  // retryPending (data thread) can never match the same buffered cloud / publish
  // the same base_name concurrently. Each base_name is published at most once per
  // stream (recent_published_ dedup).
  void onTrigger(int64_t sync_timestamp, const std::string & base_name) {
    std::lock_guard<std::mutex> match_lk(match_mtx_);
    int64_t closest_ts = 0;
    auto match = findClosest(sync_timestamp, &closest_ts);
    if (match) {
      processMatched(match, base_name, closest_ts);
      return;
    }
    // No match. Mirror handler:531-540: if the buffer's closest is NEWER than the
    // trigger, the buffer has already moved past -> drop. Otherwise (older / not
    // yet arrived) enqueue for retry.
    bool buffer_empty;
    {
      std::lock_guard<std::mutex> lk(buf_mtx_);
      buffer_empty = buffer_.empty();
    }
    if (buffer_empty) {
      // Nothing buffered yet — definitely "not arrived". Enqueue.
      enqueuePending(sync_timestamp, base_name);
      return;
    }
    const int64_t signed_diff_ns = closest_ts - sync_timestamp;
    if (signed_diff_ns > 0) {
      // closest is newer than trigger -> buffer moved past, drop (handler:533-535).
      return;
    }
    enqueuePending(sync_timestamp, base_name);
  }

private:
  // handler:91-100 — min abs-diff entry within max_time_diff (<=). Returns the
  // owned message copy on success; also reports the closest timestamp seen.
  std::shared_ptr<MsgT> findClosest(int64_t target_ns, int64_t * closest_ts_out) {
    std::lock_guard<std::mutex> lk(buf_mtx_);
    if (buffer_.empty()) return nullptr;
    const Entry * best = nullptr;
    int64_t best_abs = 0;
    for (const auto & e : buffer_) {
      int64_t d = e.timestamp_ns - target_ns;
      int64_t ad = d < 0 ? -d : d;
      if (best == nullptr || ad < best_abs) { best = &e; best_abs = ad; }
    }
    if (closest_ts_out) *closest_ts_out = best->timestamp_ns;
    if (best_abs <= max_time_diff_ns_) return best->msg;
    return nullptr;
  }

  void enqueuePending(int64_t sync_timestamp, const std::string & base_name) {
    std::lock_guard<std::mutex> lk(pend_mtx_);
    pending_.push_back(Pending{sync_timestamp, base_name});
    while (pending_.size() > kPendingMax) pending_.pop_front();  // deque(maxlen=10)
  }

  // handler:575-625 — re-match every pending trigger; republish matches, drop the
  // ones the buffer has moved past, keep the rest.
  void retryPending() {
    // Serialize the whole match-and-publish pass for this stream against onTrigger
    // (BUG 1 / DEVIATION 3). pend_mtx_ still guards only the pending queue; the
    // ordering match_mtx_ -> {pend_mtx_, buf_mtx_} is the same as onTrigger and
    // findClosest, so no lock-order inversion.
    std::lock_guard<std::mutex> match_lk(match_mtx_);
    std::vector<Pending> snapshot;
    {
      std::lock_guard<std::mutex> lk(pend_mtx_);
      if (pending_.empty()) return;
      snapshot.assign(pending_.begin(), pending_.end());
      pending_.clear();
    }
    std::vector<Pending> still_pending;
    still_pending.reserve(snapshot.size());
    for (const auto & req : snapshot) {
      int64_t closest_ts = 0;
      auto match = findClosest(req.sync_timestamp, &closest_ts);
      if (match) {
        processMatched(match, req.base_name, closest_ts);
        continue;
      }
      // Not matched. If buffer's closest is newer than the trigger, the buffer has
      // moved past this trigger -> drop. Otherwise keep waiting.
      bool empty;
      {
        std::lock_guard<std::mutex> lk(buf_mtx_);
        empty = buffer_.empty();
      }
      if (empty) { still_pending.push_back(req); continue; }
      const int64_t signed_diff_ns = closest_ts - req.sync_timestamp;
      if (signed_diff_ns > 0) {
        // newer than trigger -> drop
        continue;
      }
      still_pending.push_back(req);
    }
    if (!still_pending.empty()) {
      std::lock_guard<std::mutex> lk(pend_mtx_);
      for (const auto & p : still_pending) pending_.push_back(p);
      while (pending_.size() > kPendingMax) pending_.pop_front();
    }
  }

  // handler:680-723 (normal mode): scan-offset refresh (pointcloud, first),
  // restamp, frame_id = base_name, publish.
  //
  // BUG 1 fix — COPY-ON-PUBLISH: the buffered Entry's message (matched_msg) is
  // NEVER mutated. We make an owned copy, restamp + set frame_id on the COPY, and
  // publish the copy. The buffered Entry stays pristine so a later trigger can
  // still match the same cloud without corruption. MUST be called with match_mtx_
  // held (callers: onTrigger, retryPending) — this serializes publishes and the
  // dedup check below per stream.
  void processMatched(const std::shared_ptr<MsgT> & matched_msg, const std::string & base_name,
                      int64_t matched_ts_ns) {
    // Dedup: a given base_name is published at most once per stream regardless of
    // the onTrigger / retryPending interleaving (BUG 1).
    if (alreadyPublished(base_name)) return;

    // CALIB: flush the dense intermediates from the previous inter-trigger gap
    // BEFORE the matched frame, gated on recording (handler:686-688). The matched
    // publish itself stays UNGATED (normal-mode contract). All calib state is under
    // match_mtx_ (held by the caller), so this is serialized per stream.
    if (calibration_mode_ && recording_enabled_.load()) {
      flushCalibIntermediates(matched_ts_ns);
    }

    if (is_pointcloud_) {
      updateScanOffset(*matched_msg);  // BEFORE _apply_clock_offset (handler:700-707)
    }
    auto out = std::make_shared<MsgT>(*matched_msg);  // owned copy, buffer untouched
    applyClockOffset(*out);                           // restamp the COPY only
    out->header.frame_id = base_name;                 // frame_id on the COPY only
    markPublished(base_name);
    pub_->publish(*out);

    // CALIB: this matched frame anchors the NEXT batch of intermediates and resets
    // the rate-cap clock (handler:726-732).
    if (calibration_mode_) {
      calib_prev_base_ = base_name;
      calib_have_prev_ = true;  // once set, never unset (Python None -> not-None)
      if (store_min_interval_ns_ > 0) calib_last_stored_ns_ = matched_ts_ns;
    }
  }

  // handler_node.py:805-830 — store (here: PUBLISH as *_sync) every buffered frame in
  // (last_flush, matched_ts) as <prev_base>_<n>, rate-capped by store_min_interval.
  // Caller holds match_mtx_ (all calib state is match_mtx_-protected); buf_mtx_ is
  // taken only for the snapshot, preserving the match_mtx_ -> buf_mtx_ lock order.
  void flushCalibIntermediates(int64_t matched_ts_ns) {
    const int64_t last = calib_last_flush_ns_;
    const bool have_prev = calib_have_prev_;
    const std::string prev_base = calib_prev_base_;
    calib_last_flush_ns_ = matched_ts_ns;         // always advance (handler:814), even if no anchor
    if (!have_prev) return;                        // Python: prev_base is None -> return (first trigger)

    // Snapshot the entries strictly inside (last, matched_ts), ascending by ts.
    std::vector<std::shared_ptr<MsgT>> inter;
    std::vector<int64_t> inter_ts;
    {
      std::lock_guard<std::mutex> lk(buf_mtx_);
      for (const auto & e : buffer_) {
        if (e.timestamp_ns > last && e.timestamp_ns < matched_ts_ns) {
          inter.push_back(e.msg);
          inter_ts.push_back(e.timestamp_ns);
        }
      }
    }
    // Sort ascending by ts (buffer is arrival-ordered ~= time-ordered; sort to be exact).
    std::vector<size_t> idx(inter.size());
    for (size_t k = 0; k < idx.size(); ++k) idx[k] = k;
    std::sort(idx.begin(), idx.end(),
              [&](size_t a, size_t b) { return inter_ts[a] < inter_ts[b]; });

    int i = 0;  // enumerate start=1 over ALL selected; rate-capped frames keep their _n
    for (size_t k : idx) {
      ++i;
      const int64_t ts = inter_ts[k];
      if (store_min_interval_ns_ > 0 &&
          (ts - calib_last_stored_ns_) < store_min_interval_ns_) {
        continue;  // rate-capped (by time); the index i is still consumed (gap preserved)
      }
      calib_last_stored_ns_ = ts;
      auto out = std::make_shared<MsgT>(*inter[k]);
      applyClockOffset(*out);
      out->header.frame_id = prev_base + "_" + std::to_string(i);
      pub_->publish(*out);
    }
  }

  // Bounded recent-publish set for per-stream dedup. Caller holds match_mtx_.
  bool alreadyPublished(const std::string & base_name) const {
    for (const auto & n : recent_published_) {
      if (n == base_name) return true;
    }
    return false;
  }
  void markPublished(const std::string & base_name) {
    recent_published_.push_back(base_name);
    while (recent_published_.size() > kRecentMax) recent_published_.pop_front();
  }

  // handler:794-803 — rewrite header.stamp to orig + clock_offset + fov.
  void applyClockOffset(MsgT & msg) {
    const int64_t co = clock_offset_ns_.load();
    const int64_t fo = shared_scan_offset_ns_.load();
    if (co != 0 || fo != 0) {
      int64_t ns = stampToNs(msg.header.stamp) + co + fo;
      nsToStamp(ns, msg.header.stamp);
    }
  }

  // ---- scan offset (pointcloud handler ONLY); specialized below ----
  void updateScanOffset(const MsgT & msg);

  rclcpp::Node * node_;
  size_t buffer_cap_;
  int64_t max_time_diff_ns_;
  bool is_pointcloud_;
  const std::atomic<int64_t> & shared_scan_offset_ns_;     // read by all
  std::atomic<int64_t> & shared_scan_offset_writable_;     // written by pointcloud only
  const std::atomic<int64_t> & clock_offset_ns_;

  // --- calib intermediate-flush (handler_node.py _flush_calib_intermediates) ---
  // In calibration mode, publish the dense frames between triggers as
  // <prev_base>_<n> so the Python _store (store_all, reading *_sync) writes them.
  // ALL calib state below is touched ONLY inside processMatched, which runs under
  // match_mtx_ (onTrigger + retryPending both hold it) → serialized per stream.
  const bool calibration_mode_;
  const int64_t store_min_interval_ns_;          // 1e9/store_max_hz (0 = uncapped)
  const std::atomic<bool> & recording_enabled_;  // gates the intermediate publish
  // Python's anchor is `None` until the first matched publish. A ROS frame_id/
  // img_name can legitimately be the empty string, so we distinguish "never
  // anchored" from "empty base" with a SEPARATE flag (set once, never reset) —
  // NOT calib_prev_base_.empty(). Faithful port of `if prev_base is None`.
  bool calib_have_prev_{false};                  // false == Python prev_base is None
  std::string calib_prev_base_;                  // handler_node.py:251 (anchor base)
  int64_t calib_last_flush_ns_{0};               // handler_node.py:250
  int64_t calib_last_stored_ns_{0};              // handler_node.py:202

  std::mutex buf_mtx_;
  std::deque<Entry> buffer_;
  std::mutex pend_mtx_;
  std::deque<Pending> pending_;
  static constexpr size_t kPendingMax = 10;  // deque(maxlen=10), handler:232

  // BUG 1 / DEVIATION 3: serializes the match-and-publish path (onTrigger vs.
  // retryPending) for this stream, and guards recent_published_.
  std::mutex match_mtx_;
  std::deque<std::string> recent_published_;  // last published base_names (dedup)
  static constexpr size_t kRecentMax = 32;    // bounded ~32 (task spec)

  typename rclcpp::Subscription<MsgT>::SharedPtr sub_;
  typename rclcpp::Publisher<MsgT>::SharedPtr pub_;
};

// Default: non-pointcloud streams never compute the scan offset.
template <typename MsgT>
void BufferHandler<MsgT>::updateScanOffset(const MsgT &) {}

// PointCloud2 specialization — handler:762-792 cloned byte-for-byte.
template <>
void BufferHandler<sensor_msgs::msg::PointCloud2>::updateScanOffset(
    const sensor_msgs::msg::PointCloud2 & pc) {
  int t_off = -1, x_off = -1;
  for (const auto & f : pc.fields) {
    if (f.name == "t") t_off = static_cast<int>(f.offset);
    else if (f.name == "x") x_off = static_cast<int>(f.offset);
  }
  if (t_off < 0) return;  // no 't' field -> leave shared value unchanged

  const uint32_t ps = pc.point_step;
  if (ps == 0) return;
  const size_t n = pc.data.size() / ps;
  const uint8_t * base = pc.data.data();

  bool have = false;
  uint32_t t_min = 0, t_max = 0;
  for (size_t i = 0; i < n; ++i) {
    const uint8_t * p = base + i * ps;
    if (x_off >= 0) {
      float x;
      std::memcpy(&x, p + x_off, sizeof(float));
      if (!(std::fabs(x) > 0.01f)) continue;  // abs(x) > 0.01 valid-point filter
    }
    uint32_t t;
    std::memcpy(&t, p + t_off, sizeof(uint32_t));  // 't' is uint32, 4 bytes
    if (!have) { t_min = t_max = t; have = true; }
    else { if (t < t_min) t_min = t; if (t > t_max) t_max = t; }
  }
  if (have) {
    // (t.min() + t.max()) // 2 over valid points (handler:786). Use int64 to avoid
    // overflow; Python does int(t.min()) + int(t.max()) (Python ints, no overflow).
    int64_t off = (static_cast<int64_t>(t_min) + static_cast<int64_t>(t_max)) / 2;
    shared_scan_offset_writable_.store(off);
  }
  // else: t.size == 0 -> previous value persists.
}

class OusterSyncNode : public rclcpp::Node {
public:
  OusterSyncNode() : rclcpp::Node("ouster_sync_node") {
    // --- params ---
    main_trigger_topic_ = this->declare_parameter<std::string>(
        "main_trigger_topic", "visible_camera/image_with_metadata");
    clock_offset_topic_ = this->declare_parameter<std::string>(
        "clock_offset_topic", "/ouster/clock_offset");
    // Output suffix: live = "_sync", parallel test = "_sync_cpp" to avoid colliding
    // with the live Python _sync topics.
    const std::string suffix = this->declare_parameter<std::string>("output_suffix", "_sync");
    // max_time_diff = 0.06 s for all 5 (compositor:116).
    const double max_diff_s = this->declare_parameter<double>("max_time_diff", 0.06);
    const int64_t max_diff_ns = static_cast<int64_t>(max_diff_s * 1e9);

    // --- calibration intermediate-flush params ---
    // Default false / 0.0 => store_min_interval_ns_=0 and calibration_mode_=false
    // => ALL calib logic dead => normal mode byte-identical to today.
    calibration_mode_ = this->declare_parameter<bool>("calibration_mode", false);
    const double store_max_hz = this->declare_parameter<double>("store_max_hz", 0.0);
    // handler_node.py:200-201 — int(1e9/hz) truncated; static_cast truncates == int().
    store_min_interval_ns_ = (store_max_hz > 0.0)
        ? static_cast<int64_t>(1e9 / store_max_hz) : 0;

    // Separate callback groups so the 5 data callbacks + trigger run concurrently
    // (reentrant) under the MultiThreadedExecutor — no GIL, no serialization.
    cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // --- clock offset subscription (RELIABLE + TRANSIENT_LOCAL depth 1) ---
    auto off_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    rclcpp::SubscriptionOptions off_opts;
    off_opts.callback_group = cb_group_;
    clock_offset_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        clock_offset_topic_, off_qos,
        [this](std_msgs::msg::Float64::ConstSharedPtr m) {
          // Float64 is SECONDS; int(data*1e9) truncated toward zero (handler:449).
          clock_offset_ns_.store(static_cast<int64_t>(m->data * 1e9));
          got_clock_offset_.store(true);
        },
        off_opts);

    // --- recording_enabled subscription (calib only) ---
    // Gates the intermediate flush. QoS = RELIABLE + VOLATILE + KEEP_LAST(10),
    // identical to the Python handlers (buffer_handler_node.py:271 create_subscription
    // (..., 10)). This is the CORRECT choice: the primary acquisition GUI publisher
    // (multiespectral_ros_ac.py:99) is RELIABLE + VOLATILE, and a VOLATILE subscriber
    // is durability-compatible with BOTH that and the web_manager's TRANSIENT_LOCAL
    // publisher (ros_monitor.py:422). A TRANSIENT_LOCAL subscriber would be INCOMPATIBLE
    // with the VOLATILE GUI publisher and SILENTLY drop every toggle from it -> never
    // flush. Guarded behind calibration_mode_ so NORMAL mode creates no extra reader
    // (strict normal-mode invariance: default false => this subscription never exists).
    if (calibration_mode_) {
      rclcpp::SubscriptionOptions rec_opts;
      rec_opts.callback_group = cb_group_;
      recording_sub_ = this->create_subscription<std_msgs::msg::Bool>(
          "/Multiespectral/recording_enabled", rclcpp::QoS(rclcpp::KeepLast(10)),
          [this](std_msgs::msg::Bool::ConstSharedPtr m) {
            recording_enabled_.store(m->data);
          },
          rec_opts);
    }

    // --- handlers ---
    // image handlers (cap 40), pointcloud (cap 30); all max_time_diff 0.06.
    auto add_image = [&](const std::string & in, const std::string & out_rel) {
      image_handlers_.push_back(std::make_unique<BufferHandler<sensor_msgs::msg::Image>>(
          this, in, out_rel + suffix, 40, max_diff_ns, /*is_pc=*/false,
          scan_offset_ns_, scan_offset_ns_, clock_offset_ns_,
          calibration_mode_, store_min_interval_ns_, recording_enabled_));
    };
    // range_image removed from the pipeline: it reconstructs EXACTLY from the
    // pointcloud (range field, verified bit-for-bit), so we don't sub/match/publish
    // it — saves the /ouster/range_image ROS transport and downstream crop+store.
    // add_image("/ouster/range_image",  "ouster/range_image");
    add_image("/ouster/reflec_image", "ouster/reflec_image");
    add_image("/ouster/signal_image", "ouster/signal_image");
    add_image("/ouster/nearir_image", "ouster/nearir_image");

    pc_handler_ = std::make_unique<BufferHandler<sensor_msgs::msg::PointCloud2>>(
        this, "/ouster/points", std::string("ouster/points") + suffix, 30, max_diff_ns,
        /*is_pc=*/true, scan_offset_ns_, scan_offset_ns_, clock_offset_ns_,
        calibration_mode_, store_min_interval_ns_, recording_enabled_);

    // --- trigger subscription ---
    // Default QoS-ish: KEEP_LAST depth 10 reliable (matches the camera publisher).
    rclcpp::SubscriptionOptions trig_opts;
    trig_opts.callback_group = cb_group_;
    trigger_sub_ = this->create_subscription<multiespectral_acquire::msg::ImageWithMetadata>(
        main_trigger_topic_, rclcpp::QoS(rclcpp::KeepLast(10)),
        std::bind(&OusterSyncNode::triggerCb, this, _1), trig_opts);

    // Fail-loud watchdog: if no clock_offset arrives within 5 s of the FIRST
    // trigger, log an error (Python silently buffered at offset 0 -> ~1.78e12 ms
    // desync). This is an ADDED safety check, not in the Python.
    watchdog_ = this->create_wall_timer(std::chrono::milliseconds(500), [this]() {
      if (!first_trigger_seen_.load()) return;
      if (got_clock_offset_.load()) { watchdog_->cancel(); return; }
      const int64_t now = this->now().nanoseconds();
      if (now - first_trigger_ns_.load() > 5LL * 1000000000LL) {
        RCLCPP_ERROR(this->get_logger(),
            "[ouster_sync] NO /ouster/clock_offset received within 5 s of first trigger "
            "(topic '%s'). Lidar is being buffered at clock_offset=0 -> sync is WRONG. "
            "Is ouster_recal_node running and publishing TRANSIENT_LOCAL?",
            clock_offset_topic_.c_str());
        watchdog_->cancel();  // log once
      }
    });

    RCLCPP_INFO(this->get_logger(),
        "[ouster_sync] up. trigger='%s' clock_offset='%s' suffix='%s' max_diff=%.3fs",
        main_trigger_topic_.c_str(), clock_offset_topic_.c_str(), suffix.c_str(), max_diff_s);
  }

private:
  void triggerCb(multiespectral_acquire::msg::ImageWithMetadata::ConstSharedPtr msg) {
    if (!first_trigger_seen_.exchange(true)) {
      first_trigger_ns_.store(this->now().nanoseconds());
    }
    // sync_timestamp = camera_ts + exposure_time/2 (integer floor) — handler:519-526.
    const int64_t camera_ts = stampToNs(msg->metadata.header.stamp);
    const uint64_t exposure = msg->metadata.exposure_time;  // ns
    int64_t sync_timestamp = camera_ts;
    if (exposure > 0) sync_timestamp += static_cast<int64_t>(exposure / 2);  // floor
    const std::string base_name = msg->metadata.img_name;

    // Each of the 5 streams independently matches this trigger.
    for (auto & h : image_handlers_) h->onTrigger(sync_timestamp, base_name);
    pc_handler_->onTrigger(sync_timestamp, base_name);
  }

  std::string main_trigger_topic_;
  std::string clock_offset_topic_;

  std::atomic<int64_t> clock_offset_ns_{0};
  std::atomic<int64_t> scan_offset_ns_{0};  // shared: pointcloud writes, all read
  std::atomic<bool> got_clock_offset_{false};
  std::atomic<bool> first_trigger_seen_{false};
  std::atomic<int64_t> first_trigger_ns_{0};

  // --- calibration intermediate-flush config/state (node-owned) ---
  bool calibration_mode_{false};            // param; false => all calib logic dead
  int64_t store_min_interval_ns_{0};        // int(1e9/store_max_hz), 0 = uncapped
  // Written by recording_sub_ callback, read (const ref) by every handler's flush
  // gate. In normal mode the subscription is never created, so this stays false.
  std::atomic<bool> recording_enabled_{false};

  rclcpp::CallbackGroup::SharedPtr cb_group_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr clock_offset_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr recording_sub_;
  rclcpp::Subscription<multiespectral_acquire::msg::ImageWithMetadata>::SharedPtr trigger_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_;

  std::vector<std::unique_ptr<BufferHandler<sensor_msgs::msg::Image>>> image_handlers_;
  std::unique_ptr<BufferHandler<sensor_msgs::msg::PointCloud2>> pc_handler_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OusterSyncNode>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
