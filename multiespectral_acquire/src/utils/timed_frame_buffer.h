#ifndef TIMED_FRAME_BUFFER_H
#define TIMED_FRAME_BUFFER_H

#include <deque>
#include <cstdint>
#include <algorithm>
#include <chrono>

template <typename FrameType>
class TimedFrameBuffer {
protected:
    std::deque<FrameType> buffer_;
    size_t max_size_ = 50;
    uint64_t frames_added_ = 0;
    uint64_t frames_extracted_ = 0;
    std::chrono::steady_clock::time_point last_adjust_;

public:
    explicit TimedFrameBuffer(size_t max_size = 30) : max_size_(max_size) {
        last_adjust_ = std::chrono::steady_clock::now();
    }

    void addFrame(const FrameType& frame) {
        if (buffer_.size() >= max_size_) buffer_.pop_front();
        buffer_.push_back(frame);
        frames_added_++;
        checkAndAdapt();
    }

    bool empty() const { return buffer_.empty(); }
    size_t size() const { return buffer_.size(); }

    FrameType* findClosest(uint64_t timestamp, double max_diff = -1.0) {  
        if (buffer_.empty()) return nullptr;
        frames_extracted_++;
        checkAndAdapt();

        auto it = std::min_element(buffer_.begin(), buffer_.end(),
            [timestamp](const FrameType& a, const FrameType& b) {
                return std::abs(static_cast<int64_t>(a.timestamp - timestamp)) <
                       std::abs(static_cast<int64_t>(b.timestamp - timestamp));
            });
            
        if (it == buffer_.end()) return nullptr;
        
        double time_diff_s = std::abs(static_cast<int64_t>(it->timestamp - timestamp)) / 1e9;
        if (time_diff_s > max_diff)
        {
            std::cerr << "[TimedFrameBuffer::findClosest] Closest to " << timestamp << " (ns) is " << it->timestamp << " (ns); time difference: " << time_diff_s << " (s) is greater than interval between frames ("<<max_diff<<" (s))." << std::endl;
            return nullptr;
        }
        
        return &(*it);
    }

private:
    void checkAndAdapt() {
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - last_adjust_).count();
        
        if (elapsed < 2.0) return;
        
        if (frames_extracted_ > 5) {
            double input_rate = frames_added_ / elapsed;
            double output_rate = frames_extracted_ / elapsed;
            
            if (output_rate > 0.1) {
                max_size_ = std::max(5ul, std::min(100ul, size_t(input_rate / output_rate * 1.2)));
            }
        }
        
        // Reset contadores
        frames_added_ = 0;
        frames_extracted_ = 0;
        last_adjust_ = now;
    }
};

#endif // TIMED_FRAME_BUFFER_H
