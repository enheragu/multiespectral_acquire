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
    double alpha_offset = 0.05; // Smoothing factor for offset (lower = more conservative)
    double alpha_slope = 0.01;  // Smoothing factor for slope (increased for faster adaptation)
    int samples_count = 0;
    std::string camera_name = "DefaultCamera"; // For logging identification
    
    std::unique_ptr<std::deque<double>> recent_errors_ms;
    std::unique_ptr<std::deque<std::pair<double, int64_t>>> recent_samples; // (cam_ns, pc_ns) pairs
    static constexpr size_t max_buffer_size = 100;
    static constexpr size_t slope_recalc_window = 50; // Window for slope recalculation
    
    // Drift detection
    int consecutive_anomalies = 0;
    int anomaly_log_throttle = 0;  // Throttle excessive logging
    double last_error_trend = 0.0; // Track error trend direction
    
    TimestampCalibration() : 
        recent_errors_ms(new std::deque<double>()),
        recent_samples(new std::deque<std::pair<double, int64_t>>()) {}
    
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
        
        // DEBUG each 50 frames
        static int debug_counter = 0;
        if (++debug_counter % 50 == 0) {
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
        
        // Adaptive threshold based on recent error statistics
        double mean_error = getMeanError();
        double std_dev = getStdDevError();
        double adaptive_threshold = std::max(100.0, std::abs(mean_error) + 3.0 * std_dev);
        
        // Detect systematic drift: if error is consistently growing in one direction
        bool is_systematic_drift = false;
        if (recent_errors_ms->size() >= 10) {
            double error_trend = getErrorTrend();
            // If error is growing by more than 5ms per sample consistently, it's systematic drift
            if (std::abs(error_trend) > 5.0) {
                is_systematic_drift = true;
            }
        }
        
        // Update with exponential filter if error is not anomalous OR if systematic drift detected
        if (std::abs(error_ms) < adaptive_threshold || is_systematic_drift) {
            consecutive_anomalies = 0;  // Reset anomaly counter
            
            // Store sample for slope recalculation
            recent_samples->push_back({cam_ns, pc_ns});
            if (recent_samples->size() > slope_recalc_window) {
                recent_samples->pop_front();
            }
            
            // Update offset using exponential moving average
            offset_ns += alpha_offset * error_ns;
            samples_count++;
            
            // Recalculate slope more frequently when drift is detected
            int recalc_interval = is_systematic_drift ? 10 : 25;
            if (samples_count % recalc_interval == 0 && recent_samples->size() >= 10) {
                recalculateSlope(is_systematic_drift);
            }
            
            // Log every 100 samples
            if (samples_count % 100 == 0) {
                std::cout << "[" << camera_name << "::TimestampCalibration] Adaptive #" << samples_count 
                          << " - Offset: " << offset_ns/1e6 << " ms"
                          << ", Slope: " << slope
                          << ", Mean error: " << mean_error << " ms (±" << std_dev << " ms)" 
                          << ", Threshold: " << adaptive_threshold << " ms"
                          << (is_systematic_drift ? " [DRIFT CORRECTION ACTIVE]" : "")
                          << std::endl;
            }
        } else {
            consecutive_anomalies++;
            anomaly_log_throttle++;
            
            // Only log first few anomalies, then every 10th, to reduce log spam
            if (consecutive_anomalies <= 3 || anomaly_log_throttle % 10 == 0) {
                std::cout << "[" << camera_name << "::TimestampCalibration] Anomalous sample ignored (error: " 
                          << error_ms << " ms, threshold: " << adaptive_threshold << " ms)";
                if (consecutive_anomalies > 3) {
                    std::cout << " [" << consecutive_anomalies << " consecutive]";
                }
                std::cout << std::endl;
            }
            
            // If too many consecutive anomalies, force a slope recalculation with higher alpha
            if (consecutive_anomalies >= 20) {
                std::cout << "[" << camera_name << "::TimestampCalibration] WARNING: " << consecutive_anomalies 
                          << " consecutive anomalies - forcing recalibration" << std::endl;
                
                // Add current sample despite being anomalous to allow recalibration
                recent_samples->push_back({cam_ns, pc_ns});
                if (recent_samples->size() > slope_recalc_window) {
                    recent_samples->pop_front();
                }
                
                // Force aggressive recalibration
                recalculateSlope(true);
                consecutive_anomalies = 0;
                anomaly_log_throttle = 0;
            }
        }
    }
    
    /**
     * @brief Calculate error trend (slope of error over recent samples)
     * @return Positive = error growing, negative = error shrinking
     */
    double getErrorTrend() const {
        if (recent_errors_ms->size() < 5) return 0.0;
        
        // Simple linear regression on recent errors to detect trend
        double sum_x = 0, sum_y = 0, sum_xy = 0, sum_xx = 0;
        int n = std::min((size_t)20, recent_errors_ms->size());  // Use last 20 samples
        
        auto it = recent_errors_ms->end();
        std::advance(it, -n);
        
        for (int i = 0; i < n; ++i, ++it) {
            sum_x += i;
            sum_y += *it;
            sum_xy += i * (*it);
            sum_xx += i * i;
        }
        
        return (n * sum_xy - sum_x * sum_y) / (n * sum_xx - sum_x * sum_x);
    }
    
    /**
     * @brief Recalculate slope using linear regression on recent samples
     * @param aggressive If true, apply slope change more aggressively (for drift correction)
     */
    void recalculateSlope(bool aggressive = false) {
        if (recent_samples->size() < 10) return;
        
        double sum_x = 0, sum_y = 0, sum_xy = 0, sum_xx = 0;
        int n = recent_samples->size();
        
        for (const auto& sample : *recent_samples) {
            double cam_ns = sample.first;
            int64_t pc_ns = sample.second;
            sum_x += cam_ns;
            sum_y += pc_ns;
            sum_xy += cam_ns * pc_ns;
            sum_xx += cam_ns * cam_ns;
        }
        
        double new_slope = (n * sum_xy - sum_x * sum_y) / (n * sum_xx - sum_x * sum_x);
        double new_offset = (sum_y - new_slope * sum_x) / n;
        
        // Use higher alpha when in aggressive mode (drift correction)
        double effective_alpha = aggressive ? 0.1 : alpha_slope;
        
        // Apply exponential smoothing to slope changes
        double old_slope = slope;
        slope = (1.0 - effective_alpha) * slope + effective_alpha * new_slope;
        
        // Update offset more aggressively when slope changes
        offset_ns = (1.0 - effective_alpha) * offset_ns + effective_alpha * new_offset;
        
        // Log slope changes if significant
        if (std::abs(old_slope - slope) > 0.0001) {
            std::cout << "[" << camera_name << "::TimestampCalibration] Slope adjusted: " 
                      << old_slope << " -> " << slope 
                      << " (delta: " << (slope - old_slope) << ")"
                      << (aggressive ? " [AGGRESSIVE]" : "") << std::endl;
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
    std::cout << "              - Adaptive calibration ENABLED (alpha_offset=" << cal.alpha_offset 
              << ", alpha_slope=" << cal.alpha_slope << ")" << std::endl;
    
    if(r_squared < 0.99) {
        std::cerr << "[" << camera_name << "::TimestampCalibration] WARNING: Low R² (" << r_squared 
                  << "). Calibration may not be reliable." << std::endl;
    }
    
    return cal;
}

#endif // TIMESTAMP_CALIBRATION_H
