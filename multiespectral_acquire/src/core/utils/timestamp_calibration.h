#ifndef TIMESTAMP_CALIBRATION_H
#define TIMESTAMP_CALIBRATION_H

#include <deque>
#include <memory>
#include <cmath>
#include <iostream>

/**
 * @brief Adaptive timestamp calibration for cameras without PTP support
 * 
 * Calibrates camera hardware timestamps to PC time using linear regression
 * and exponential moving average for continuous adaptation.
 */
class TimestampCalibration {
public:
    int64_t offset_ns = 0;
    double slope = 1.0;
    
    bool initialized = false;
    double alpha = 0.05; // Smoothing factor (lower = more conservative)
    int samples_count = 0;
    std::string camera_name = "DefaultCamera"; // For logging identification
    
    std::unique_ptr<std::deque<double>> recent_errors_ms;
    static constexpr size_t max_buffer_size = 100;
    
    TimestampCalibration() : recent_errors_ms(new std::deque<double>()) {}
    
    // Movement constructor
    TimestampCalibration(TimestampCalibration&& other) noexcept = default;
    // Movement assignment operator
    TimestampCalibration& operator=(TimestampCalibration&& other) noexcept = default;
    
    /**
     * @brief Update calibration with new camera/PC timestamp pair
     * @param cam_ticks Camera timestamp in ticks
     * @param pc_ns PC timestamp in nanoseconds
     * @param tick_freq Camera tick frequency (ticks/second)
     */
    void updateWithSample(int64_t cam_ticks, int64_t pc_ns, int64_t tick_freq) {
        if (!initialized) return;
        
        double cam_ns = cam_ticks * 1e9 / tick_freq;
        double predicted_pc_ns = offset_ns + slope * cam_ns;
        
        double error_ns = pc_ns - predicted_pc_ns;
        double error_ms = error_ns / 1e6;
        
        // DEBUG each 10 frames
        static int debug_counter = 0;
        if (++debug_counter % 10 == 0) {
            std::cout << "[" << camera_name << "::TimestampCalibration] cam_ticks=" << cam_ticks 
                    << ", cam_ns=" << cam_ns/1e9 << "s"
                    << ", pc_ns=" << pc_ns/1e9 << "s"
                    << ", offset=" << offset_ns/1e6 << "ms"
                    << ", slope=" << slope
                    << ", error=" << error_ms << "ms" << std::endl;
        }
        
        recent_errors_ms->push_back(error_ms);
        if (recent_errors_ms->size() > max_buffer_size) {
            recent_errors_ms->pop_front();
        }
        
        // Update with exponential filter only if error is not anomalous (< 100ms)
        if (std::abs(error_ms) < 100.0) {
            offset_ns += alpha * error_ns;
            samples_count++;
            
            // Log every 100 samples
            if (samples_count % 100 == 0) {
                double mean_error = 0;
                for (double e : *recent_errors_ms) mean_error += e;
                mean_error /= recent_errors_ms->size();
                
                double std_dev = 0;
                for (double e : *recent_errors_ms) {
                    std_dev += (e - mean_error) * (e - mean_error);
                }
                std_dev = std::sqrt(std_dev / recent_errors_ms->size());
                
                std::cout << "[" << camera_name << "::TimestampCalibration] Adaptive #" << samples_count 
                          << " - Offset: " << offset_ns/1e6 << " ms"
                          << ", Mean error: " << mean_error << " ms (±" << std_dev << " ms)" 
                          << std::endl;
            }
        } else {
            std::cout << "[" << camera_name << "::TimestampCalibration] Anomalous sample ignored (error: " << error_ms << " ms)" << std::endl;
        }
    }
    
    /**
     * @brief Get mean error over recent samples
     */
    double getMeanError() const {
        if (recent_errors_ms->empty()) return 0.0;
        double sum = 0;
        for (double e : *recent_errors_ms) sum += e;
        return sum / recent_errors_ms->size();
    }

    /**
     * @brief Get standard deviation of error over recent samples
     */
    double getStdDevError() const {
        if (recent_errors_ms->size() < 2) return 0.0;
        double mean = getMeanError();
        double sum_sq = 0;
        for (double e : *recent_errors_ms) {
            sum_sq += (e - mean) * (e - mean);
        }
        return std::sqrt(sum_sq / recent_errors_ms->size());
    }
    
    /**
     * @brief Print final calibration statistics
     */
    void printFinalStats() const {
        if (!initialized || samples_count == 0) return;
        std::cout << "[" << camera_name << "::TimestampCalibration] Final statistics:" << std::endl;
        std::cout << "              - Samples processed: " << samples_count << std::endl;
        std::cout << "              - Final offset: " << offset_ns/1e6 << " ms" << std::endl;
        std::cout << "              - Mean error: " << getMeanError() << " ms" << std::endl;
        std::cout << "              - Standard deviation: " << getStdDevError() << " ms" << std::endl;
    }
};

/**
 * @brief Perform initial timestamp calibration using linear regression
 * @param camera_times Vector of camera timestamps (ticks)
 * @param pc_times Vector of PC timestamps (nanoseconds)
 * @param tick_frequency Camera tick frequency (ticks/second)
 * @return Initialized TimestampCalibration object
 */
inline TimestampCalibration performInitialCalibration(
    const std::vector<int64_t>& camera_times,
    const std::vector<int64_t>& pc_times,
    int64_t tick_frequency,
    const std::string& camera_name = "DefaultCamera")
{
    if (camera_times.size() != pc_times.size() || camera_times.empty()) {
        std::cerr << "[TimestampCalibration] Invalid input vectors" << std::endl;
        return TimestampCalibration();
    }
    
    // Linear regression: pc_time = slope * cam_time + offset
    double sum_x = 0, sum_y = 0, sum_xy = 0, sum_xx = 0;
    int n = camera_times.size();
    
    for(size_t i = 0; i < camera_times.size(); i++) {
        double cam_ns = camera_times[i] * 1e9 / tick_frequency;
        sum_x += cam_ns; 
        sum_y += pc_times[i];
        sum_xy += cam_ns * pc_times[i]; 
        sum_xx += cam_ns * cam_ns;
    }
    
    TimestampCalibration cal;
    cal.camera_name = camera_name;
    cal.slope = (n * sum_xy - sum_x * sum_y) / (n * sum_xx - sum_x * sum_x);
    cal.offset_ns = (sum_y - cal.slope * sum_x) / n;
    cal.initialized = true;
    
    // Calculate R² and max error
    double mean_y = sum_y / n;
    double ss_tot = 0, ss_res = 0;
    double max_error_ms = 0;
    
    for(size_t i = 0; i < camera_times.size(); i++) {
        double cam_ns = camera_times[i] * 1e9 / tick_frequency;
        double predicted = cal.offset_ns + cal.slope * cam_ns;
        double error = pc_times[i] - predicted;
        
        ss_res += error * error;
        ss_tot += (pc_times[i] - mean_y) * (pc_times[i] - mean_y);
        
        double error_ms = std::abs(error) / 1e6;
        max_error_ms = std::max(max_error_ms, error_ms);
    }
    
    double r_squared = 1.0 - (ss_res / ss_tot);
    
    std::cout << "[" << camera_name << "::TimestampCalibration] Initial calibration results:" << std::endl;
    std::cout << "              - Slope: " << cal.slope << std::endl;
    std::cout << "              - Offset: " << cal.offset_ns/1e6 << " ms" << std::endl;
    std::cout << "              - R²: " << r_squared << std::endl;
    std::cout << "              - Max error: " << max_error_ms << " ms" << std::endl;
    std::cout << "              - Adaptive calibration ENABLED (alpha=" << cal.alpha << ")" << std::endl;
    
    if(r_squared < 0.99) {
        std::cerr << "[" << camera_name << "::TimestampCalibration] WARNING: Low R² (" << r_squared 
                  << "). Calibration may not be reliable." << std::endl;
    }
    
    return cal;
}

#endif // TIMESTAMP_CALIBRATION_H
