# multiespectral_acquire

ROS 2 package for synchronized multi-sensor data acquisition: GigE camera drivers, buffer-based temporal synchronization, FOV crop, and structured disk storage.

The package is **hardware-agnostic** — any combination of cameras and sensors can be configured via launch files. Two configurations are active:

| Configuration | Trigger source | Followers | LiDAR pipeline |
|---|---|---|---|
| **Multiespectral** | Basler RGB (1 Hz) | FLIR LWIR (software-cal), Ouster (internal-osc + recal, cropped), GNSS, odometry, DHT22 | sync → FOV crop → store |
| **Fisheye** | Basler frontal (1 Hz) | Basler rear (4 Hz), Ouster (raw), GNSS, odometry | sync → store |

For the full HITOS hardware deployment (network topology, power, services) see [`hitos_setup/README.md`](../../hitos_setup/README.md).

---

## Nodes

### Camera drivers (C++)

| Executable | Camera | SDK |
|------------|--------|-----|
| `basler_camera_handler` | Basler acA1600-60gc (RGB) | Pylon |
| `flir_camera_handler` | FLIR A68 (LWIR thermal) | Spinnaker |
| `dummy_camera_handler` | Synthetic test pattern | — |

Each driver publishes `ImageWithMetadata` at a configurable rate via software trigger.

### Crop nodes (C++)

| Executable | Purpose |
|------------|---------|
| `pointcloud_crop_node` | Crops 3D point cloud to a configurable FOV (angular or pixel-based) |
| `image_crop_node` | Crops the four Ouster 2D image projections to the same FOV |

### Buffer compositor (Python)

`scripts/buffer_compositor_node.py` runs all buffer handler instances in a single process, saving ~15 × 50 MB vs one process per handler.

`scripts/buffer_handler_node.py` is the per-sensor logic. It has two modes:

- **store_all** (no `main_topic`): publishes to `<topic>_sync` and stores every frame. The ring buffer exists but is never populated — no memory cost.
- **sync** (with `main_topic`): aligns follower data to the main trigger timestamp. On each trigger it searches the ring buffer for the nearest match within `max_time_diff`. If not yet available it queues the request and retries on each new incoming message. Missed matches are dropped and the buffer grows automatically (bounded) when the main trigger consistently arrives late.

#### Buffer sizing

Each sync handler has two configurable parameters:

| Parameter | Role |
|-----------|------|
| `buffer_initial_size` | Starting `deque` maxlen — low idle RAM footprint |
| `buffer_max_size` | Ceiling for the auto-grow mechanism |

The buffer needs to hold at least `rate_hz / trigger_hz` frames to cover one trigger interval, plus margin for periods where the master trigger is temporarily absent (e.g. camera respawn). Values set in `buffer_compositor_node.py`:

| Handler | Topic rate | `initial` | `max` | Notes |
|---------|-----------|-----------|-------|-------|
| `buffer_lwir` | 30 Hz | 30 | 90 | 3 s coverage at 30 Hz |
| `buffer_*_sync` (Ouster 2D images) | 10 Hz | 15 | 40 | hard safety cap; the active window is state-machine-managed |
| `buffer_pointcloud_sync` | 10 Hz | 15 | 30 | dense cloud (`organized: false`) so message size is ≪ a full scan |
| `buffer_odom` | ~50 Hz | 60 | 120 | Messages are small (~200 B) |
| `buffer_gnss`, `buffer_dht22` | 5 Hz / 2 Hz | 20 | 60 | Byte-level messages |

> **RPi 5 note:** running all handlers in one process at the above limits kept RSS under 550 MB in practice vs >1.4 GB with the previous flat cap of 120 for all handlers. The main driver was `buffer_pointcloud_sync`; running the Ouster in `organized: false` (dense cloud, valid points only — see the hub README) keeps that handler's footprint small.

### Camera crash recovery

Both camera drivers run with `respawn=True` (10 s delay), so a crash is recovered automatically. Combined with the GigE Vision heartbeat timeout — both drivers explicitly set `GevHeartbeatTimeout = 3000 ms` on camera open — the camera releases its control channel within 3 s of a crash and the next respawn connects cleanly.

#### Unrecoverable corner case: permanent control-channel lock

If Spinnaker's debug mode was active in a previous session it sets `GevGVCPHeartbeatDisable = true` on the FLIR A68, disabling the heartbeat entirely. The camera then stays locked to that dead session indefinitely — all subsequent `Init()` calls fail with `[-1005] ACCESS_DENIED` regardless of how long you wait. The respawn loop cannot escape this state.

**Only recovery: power-cycle the FLIR camera** (cut PoE or cycle the switch port). The next driver start re-enables the heartbeat and sets the 3 s timeout, so the issue will not recur within that session.

---

## Timestamp synchronization

All sensors synchronize to the **visible camera (Basler)** as the main trigger. The sync point is the exposure midpoint:

$$t_\text{sync} = t_\text{camera} + \frac{t_\text{exposure}}{2}$$

### PTP mode (intended) — and why it falls back

PTP (IEEE 1588) was the intended design: each sensor locks its hardware clock to the RPi 5 grandmaster (`ptp4l` on `eth0`) so timestamps share the host time domain with no software correction. **On this rig PTP never locks** — the RPi 5 master is verified-perfect (pcap analysis), but neither the FLIR nor the Ouster reaches `Locked` behind the Mokerlink switch (see `hitos_setup` → Timestamp synchronization). So in practice:

- **FLIR** (`lwir_use_ptp=true`) is **lock-aware**: at `beginAcquisition()` it reads `ptpServoStatus` and probes a frame against UTC; only if it is genuinely `Locked` does it trust the camera clock — otherwise (the current case) it falls back to **software calibration**.
- **Ouster** runs `TIME_FROM_INTERNAL_OSC` (a free-running crystal); `hitos_setup/ouster_recal_node` maps it to wall-clock from the 100 Hz IMU — **not** PTP.

### Software calibration (Basler)

The Basler acA1600-60gc uses GigE trigger timestamps that drift from `CLOCK_REALTIME`. In the HITOS configuration `visible_use_ptp=false` so software calibration is always active:

1. **Initial calibration** (startup): N samples (30 at 1 Hz), least-squares linear fit:
   $$t_\text{PC} = \text{offset} + \text{slope} \times t_\text{camera}$$
2. **Online refinement**: EMA on offset (α = 0.05), slope recalculated every 25 samples on a 50-sample sliding window.
3. **NTP step detection**: 3+ consecutive same-sign anomalies trigger an immediate 80% offset correction instead of waiting for EMA convergence.
4. **Drift detection**: error trend > 2 ms/sample switches to aggressive recalculation (every 10 samples, α = 0.1).

---

## Launch files

| Launch file | Description |
|-------------|-------------|
| `multiespectral_launch.py` | Main entry point: camera drivers + FOV crop + buffer compositor |
| `cameras_only.launch.py` | Camera drivers only (Basler visible + FLIR LWIR) |
| `capture_sync.launch.py` | LiDAR crop nodes + buffer compositor (capture synchronization) |
| `multiespectral_buffer_handlers.launch.py` | Buffer compositor node (included by `capture_sync` / main) |
| `multiespectral_lidar_crop.launch.py` | LiDAR FOV crop nodes (included by `capture_sync` / main) |

### Key parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `session_folder` | `test_session` | Output subfolder name |
| `dataset_output_path` | `/tmp/multiespectral_data` | Base storage path. On HITOS `hitos_sync.service` overrides this to the external HDD (`/media/arvc/DATASETS/images_eeha`) |
| `output_frame_rate` | `1` | Acquisition rate (Hz) |
| `calibration_mode` | `false` | `true` → store every frame uncropped, prefix the session dir `calib_` (vs `mult_`) and write raw `.npy`; for LiDAR↔camera calibration |
| `visible_use_ptp` | `false` | Basler has no PTP — always software calibration |
| `lwir_use_ptp` | `true` | Lock-aware PTP for the FLIR; falls back to software calibration when not `Locked` (the case on this rig) |
| Buffer `max_time_diff` | `0.1 s` (LWIR), `0.3 s` (LiDAR) | Max sync tolerance |

---

## Messages

| Message | Fields |
|---------|--------|
| `ImageWithMetadata` | `sensor_msgs/Image image` + `ImageMetadata metadata` |
| `ImageMetadata` | `camera_timestamp`, `exposure_time`, `half_exposure_timestamp`, `frame_counter`, `gain`, `width`, `height`, `pixel_format`, `timetag`, `img_name`, `img_pair_name`, `dataset_name` |

---

## Ouster LiDAR images

The Ouster driver publishes four 2D image projections per scan, all cropped by `image_crop_node` to the cameras' FOV before storage:

| Type | Topic | Content |
|------|-------|---------|
| Range | `/ouster/range_image` | Per-point distance |
| Reflectivity | `/ouster/reflec_image` | Laser return intensity |
| Signal | `/ouster/signal_image` | Photon count |
| Near-IR | `/ouster/nearir_image` | Ambient infrared |

---

## Driver architecture

<details>
<summary>Core/ROS separation (design notes)</summary>

The C++ code splits into a ROS-independent core and a thin ROS 2 layer so vendor SDK logic is fully testable without a ROS environment:

```
src/
├── camera_handler_node.cpp     ← ROS 2 thin layer (params, publishers, timer)
├── image_crop_node.cpp
├── pointcloud_crop_node.cpp
├── core/                       ← No ROS dependencies
│   ├── camera_drivers/
│   │   ├── camera_adapter.h/.cpp   ← Abstract base + free-function API
│   │   ├── basler_adapter.cpp      ← Pylon SDK
│   │   ├── flir_adapter.cpp        ← Spinnaker SDK
│   │   └── dummy_adapter.cpp       ← Test stub
│   └── utils/
│       ├── logging_utils.h         ← Pure-virtual Logger
│       ├── image_metadata.h/.cpp   ← ImageMetadata + YAML serialization
│       ├── timed_frame_buffer.h    ← Self-adjusting ring buffer
│       └── timestamp_calibration.h ← PTP / software calibration
└── ros_utils/
    └── ros_logger.h            ← RosLogger : Logger → RCLCPP_INFO/WARN/etc.
```

- **Compile-time vendor selection**: CMake links exactly one of `basler_adapter.cpp`, `flir_adapter.cpp`, or `dummy_adapter.cpp` per executable.
- **Logger abstraction**: `core/utils/logging_utils.h` defines a pure-virtual `Logger`. The ROS layer injects `RosLogger` via `CameraAdapter::setLogger()`.
- **`ImageMetadata` bridge**: plain C++ struct with a `ROSTimeNowCallback` slot so the ROS layer can inject `rclcpp::Clock::now()` without the core including any ROS header.

</details>
