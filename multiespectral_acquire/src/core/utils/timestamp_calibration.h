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
            // If error is growing by more than 2ms per sample consistently, it's systematic drift
            if (std::abs(error_trend) > 2.0) {
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
            
            // Recalculate slope more frequently when drift is detected or during early convergence
            int recalc_interval = (is_systematic_drift || samples_count <= 100) ? 10 : 25;
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
            
            // Detect clock step: 3+ consecutive anomalies with same error sign
            // This happens when NTP/chrony adjusts the system clock suddenly
            if (consecutive_anomalies >= 3 && recent_errors_ms->size() >= 3) {
                int check_count = std::min((size_t)consecutive_anomalies, recent_errors_ms->size());
                check_count = std::min(check_count, 5);
                
                bool all_same_sign = true;
                double ref_sign = (recent_errors_ms->back() > 0) ? 1.0 : -1.0;
                auto it = recent_errors_ms->end();
                std::advance(it, -check_count);
                for (int i = 0; i < check_count; ++i, ++it) {
                    if ((*it > 0 ? 1.0 : -1.0) != ref_sign) {
                        all_same_sign = false;
                        break;
                    }
                }
                
                if (all_same_sign) {
                    std::cout << "[" << camera_name << "::TimestampCalibration] Clock step detected ("
                              << consecutive_anomalies << " consecutive anomalies, error: " 
                              << error_ms << " ms). Adjusting offset immediately." << std::endl;
                    
                    // Apply direct offset correction (80% to avoid overcorrection)
                    offset_ns += 0.8 * error_ns;
                    
                    // Add sample for future slope recalculation
                    recent_samples->push_back({cam_ns, pc_ns});
                    if (recent_samples->size() > slope_recalc_window) {
                        recent_samples->pop_front();
                    }
                    
                    consecutive_anomalies = 0;
                    anomaly_log_throttle = 0;
                    recent_errors_ms->clear();
                    
                    // Skip the old 20-anomaly fallback since we handled it
                    return;
                }
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
        
        int n = recent_samples->size();
        
        // Center data to avoid numerical precision loss with large epoch values
        double mean_x = 0, mean_y = 0;
        for (const auto& sample : *recent_samples) {
            mean_x += sample.first;
            mean_y += sample.second;
        }
        mean_x /= n;
        mean_y /= n;
        
        double sum_xy = 0, sum_xx = 0;
        for (const auto& sample : *recent_samples) {
            double dx = sample.first - mean_x;
            double dy = (double)sample.second - mean_y;
            sum_xy += dx * dy;
            sum_xx += dx * dx;
        }
        
        if (sum_xx < 1e-6) return; // Not enough variance
        double new_slope = sum_xy / sum_xx;
        double new_offset = mean_y - new_slope * mean_x;
        
        // Decaying alpha: trust regression more in early samples (slope needs bigger corrections),
        // converge to alpha_slope as calibration stabilizes.
        // At samples_count=25 → ~0.15, at 100 → ~0.04, at 500 → ~0.01
        double base_alpha = aggressive ? 0.15 : std::max((double)alpha_slope, 4.0 / (samples_count + 25.0));
        
        // Apply exponential smoothing to slope changes
        double old_slope = slope;
        slope = (1.0 - base_alpha) * slope + base_alpha * new_slope;
        
        // Update offset consistently with slope
        offset_ns = (1.0 - base_alpha) * offset_ns + base_alpha * new_offset;
        
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
    // Center data to avoid numerical precision loss with large epoch timestamps
    int n = camera_times.size();
    
    double mean_x = 0, mean_y = 0;
    for(size_t i = 0; i < camera_times.size(); i++) {
        mean_x += camera_times[i] * 1e9 / tick_frequency;
        mean_y += pc_times[i];
    }
    mean_x /= n;
    mean_y /= n;
    
    double sum_xy = 0, sum_xx = 0;
    for(size_t i = 0; i < camera_times.size(); i++) {
        double dx = camera_times[i] * 1e9 / tick_frequency - mean_x;
        double dy = (double)pc_times[i] - mean_y;
        sum_xy += dx * dy;
        sum_xx += dx * dx;
    }
    
    TimestampCalibration cal;
    cal.camera_name = camera_name;
    
    double regression_slope = sum_xy / sum_xx;
    cal.offset_ns = mean_y - regression_slope * mean_x;
    
    // The initial regression over a short window (~7s) has poor slope precision
    // due to jitter (~30ms over 7.5s → ~0.004 error). For a hardware crystal oscillator
    // the true slope is very close to 1.0 (typical drift <50ppm = 0.00005).
    // Use slope=1.0 as initial value and let the adaptive calibration (50s window)
    // converge to the true slope with much better precision.
    // Recalculate offset with forced slope=1.0 for consistency.
    cal.slope = 1.0;
    cal.offset_ns = mean_y - cal.slope * mean_x;
    cal.initialized = true;
    
    // Calculate R² and max error
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
    std::cout << "              - Slope: " << cal.slope << " (regression: " << regression_slope << ")" << std::endl;
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
