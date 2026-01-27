/**
 * This file is a wrapper of Basler API for GeniCam camera acA 1600-60gc, exposes a common API to be wrapped into a
 * ROS node.
 */
#include <memory>
#include <thread>
#include <chrono>
#include <cmath>
#include <deque>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

// Basler API
#include <pylon/PylonIncludes.h>
#include <pylon/BaslerUniversalInstantCamera.h>

#include "camera_adapter.h"
#include "utils/image_metadata.h"

// Check if available and readable and writable; and combinations of them
#define CHECK_A(node){if (!IsAvailable(node)) \
{ \
    std::cout << "[BaslerAdapter::" << __func__ << "] " << #node << " is not available." << std::endl; \
    return false; \
}}
#define CHECK_R(node){if (!IsReadable(node)) \
{ \
    std::cout << "[BaslerAdapter::" << __func__ << "] " << #node << " is not readable." << std::endl; \
    return false; \
}}
#define CHECK_W(node){if (!IsWritable(node)) \
{ \
    std::cout << "[BaslerAdapter::" << __func__ << "] " << #node << " is not writable." << std::endl; \
    return false; \
}}
#define CHECK_AR(node) {CHECK_A(node); CHECK_R(node); }
#define CHECK_AW(node) {CHECK_A(node); CHECK_W(node); }
#define CHECK_ARW(node) {CHECK_A(node); CHECK_R(node); CHECK_W(node); }

#define CHECK_POINTER(pointer){if (!pointer) \
{ \
    std::cout << "[BaslerAdapter::" << __func__ << "] " << #pointer << " is not available." << std::endl; \
    return false; \
}}

// Reference to basler camera to be handled
std::unique_ptr<Pylon::CBaslerUniversalInstantCamera> pBasler;
std::string camera_name = "Default:acA1600-60gc";
int64_t tick_frequency = 1000000; // ticks per second default 1 MHz


struct TimestampCalibration {
    int64_t offset_ns = 0;
    double slope = 1.0;
    
    bool initialized = false;
    double alpha = 0.05; // Smoothing factor (lower = more conservative)
    int samples_count = 0;
    
    std::unique_ptr<std::deque<double>> recent_errors_ms;
    static constexpr size_t max_buffer_size = 100;
    
    TimestampCalibration() : recent_errors_ms(new std::deque<double>()) {}
    // Movement constructor
    TimestampCalibration(TimestampCalibration&& other) noexcept = default;
    // Movement assignment operator
    TimestampCalibration& operator=(TimestampCalibration&& other) noexcept = default;
    
    void updateWithSample(int64_t cam_ticks, int64_t pc_ns, int64_t tick_freq) {
        if (!initialized) return;
        
        double cam_ns = cam_ticks * 1e9 / tick_freq;
        double predicted_pc_ns = offset_ns + slope * cam_ns;
        
        double error_ns = pc_ns - predicted_pc_ns;
        double error_ms = error_ns / 1e6;
        
        // DEBUG each 10 frames updated
        static int debug_counter = 0;
        if (++debug_counter % 10 == 0) {
            std::cout << "[DEBUG] cam_ticks=" << cam_ticks 
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
        
        // Update with exponential filter only if error is not anomalous (< 50ms)
        if (std::abs(error_ms) < 50.0) {
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
                
                std::cout << "[TimestampCalibration::updateWithSample] Adaptive #" << samples_count 
                          << " - Offset: " << offset_ns/1e6 << " ms"
                          << ", Mean error: " << mean_error << " ms (±" << std_dev << " ms)" 
                          << std::endl;
            }
        } else {
            std::cout << "[TimestampCalibration::updateWithSample] Anomalous sample ignored (error: " << error_ms << " ms)" << std::endl;
        }
    }
    
    double getMeanError() const {
        if (recent_errors_ms->empty()) return 0.0;
        double sum = 0;
        for (double e : *recent_errors_ms) sum += e;
        return sum / recent_errors_ms->size();
    }

    double getStdDevError() const {
        if (recent_errors_ms->size() < 2) return 0.0;
        double mean = getMeanError();
        double sum_sq = 0;
        for (double e : *recent_errors_ms) {
            sum_sq += (e - mean) * (e - mean);
        }
        return std::sqrt(sum_sq / recent_errors_ms->size());
    }
};

// Global variable to store calibration
TimestampCalibration g_calibration;

/**
 * @brief Get name of the camera for logging purposes
 */
std::string getName()
{
    if(pBasler)
    {
        camera_name = std::string("Basler ") + std::string(pBasler->GetDeviceInfo().GetModelName());
    }
    else
    {
        std::cout << "[BaslerAdapter::getName] pBasler pointer is not available." << std::endl;
    }
    return camera_name;
}

/**
 * @brief Return image type to store it correctly
 */
std::string getType()
{
    return "visible";
}

TimestampCalibration calibrateTimestamps(int num_samples = 10) {
    std::vector<int64_t> camera_times, pc_times;
    
    std::cout << "[BaslerAdapter::calibrateTimestamps] Waiting for camera stabilization (2 seconds)..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    std::cout << "[BaslerAdapter::calibrateTimestamps] Capturing " << num_samples << " samples for calibration..." << std::endl;
    
    for(int i = 0; i < num_samples; i++) {
        auto pc_start = std::chrono::system_clock::now();
        
        // Trigger + capture
        pBasler->ExecuteSoftwareTrigger();
        Pylon::CGrabResultPtr ptrGrabResult;
        pBasler->RetrieveResult(1000, ptrGrabResult);
        
        if(ptrGrabResult->GrabSucceeded()) {
            int64_t cam_ticks = ptrGrabResult->GetTimeStamp();
            auto pc_end = std::chrono::system_clock::now();
            
            // Average PC time (reduce jitter)
            int64_t pc_ns = (pc_start.time_since_epoch().count() + 
                           pc_end.time_since_epoch().count()) / 2;
            
            camera_times.push_back(cam_ticks);
            pc_times.push_back(pc_ns);
            
            std::cout << "[BaslerAdapter::calibrateTimestamps] Sample " << (i+1) << "/" << num_samples 
                      << " - Cam ticks: " << cam_ticks << ", PC ns: " << pc_ns << std::endl;
        }
        else
        {
            std::cout << "[BaslerAdapter::calibrateTimestamps] Error capturing sample " << (i+1) << std::endl;
        }
        
        // Space out samples (except the last one)
        if(i < num_samples - 1) {
            std::this_thread::sleep_for(std::chrono::milliseconds(250));
        }
    }
    
    if(camera_times.size() < 3) {
        std::cerr << "[BaslerAdapter::calibrateTimestamps] ERROR: Not enough valid samples (" 
                  << camera_times.size() << "). Using default calibration." << std::endl;
        TimestampCalibration default_cal;
        default_cal.slope = 1.0;
        default_cal.offset_ns = 0;
        default_cal.initialized = true;
        return default_cal;
    }
    
    // Regresión lineal: pc_time = slope * cam_time + offset
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
    cal.slope = (n * sum_xy - sum_x * sum_y) / (n * sum_xx - sum_x * sum_x);
    cal.offset_ns = (sum_y - cal.slope * sum_x) / n;
    cal.initialized = true;
    
    // Calcular R² correctamente
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
    
    std::cout << "[BaslerAdapter::calibrateTimestamps] Initial results:" << std::endl;
    std::cout << "              - Slope: " << cal.slope << std::endl;
    std::cout << "              - Offset: " << cal.offset_ns/1e6 << " ms" << std::endl;
    std::cout << "              - R²: " << r_squared << std::endl;
    std::cout << "              - Max error: " << max_error_ms << " ms" << std::endl;
    std::cout << "              - Adaptive calibration ENABLED (alpha=" << cal.alpha << ")" << std::endl;
    
    if(r_squared < 0.99) {
        std::cerr << "[BaslerAdapter::calibrateTimestamps] WARNING: Low R² (" << r_squared 
                  << "). Calibration may not be reliable." << std::endl;
    }
    
    return cal;
}

/**
 * @brief Function that handle all Pylon and Camera initializacion and configuration.
 * @param frame_rate frames per second to capture with camera
 * @return true or false depending on image acquisition
 */
bool initCamera(int frame_rate, std::string camera_ip)
{
    try
    {
        // Before using any pylon methods, the pylon runtime must be initialized. 
        std::cout << "[BaslerAdapter::initCamera] Initialize Pylon runtime for basler camera (frame_rate " << frame_rate << "; camera_ip " << camera_ip << ")." << std::endl;
        Pylon::PylonInitialize();
        
        // This takes first available
        pBasler = std::unique_ptr<Pylon::CBaslerUniversalInstantCamera>(new Pylon::CBaslerUniversalInstantCamera(Pylon::CTlFactory::GetInstance().CreateFirstDevice()));

        if (!pBasler)
        {
            std::cerr << "[BaslerAdapter::initCamera] Camera with configured IP ("<<camera_ip<<") was not found."  << std::endl;
            return false;
        }
        else
        {
            std::cerr << "[BaslerAdapter::initCamera] Opening camera with: " << std::endl;
            std::cout << "\t\t· Model Name " << pBasler->GetDeviceInfo().GetModelName() << std::endl;
            std::cout << "\t\t· Friendly Name: " << pBasler->GetDeviceInfo().GetFriendlyName() << std::endl;
            std::cout << "\t\t· Current IP Addr: " << pBasler->GevCurrentIPAddress.ToStringOrDefault("<not readable>") << std::endl;
            std::cout << "\t\t· Requested IP: " << camera_ip << std::endl;
        }

        pBasler->Open();

        // Enable Auto Exposure (set to Continuous mode)
        CHECK_ARW(pBasler->ExposureAuto)
        std::cout << "[BaslerAdapter::initCamera] Autoexposure enabled in continuous mode." << std::endl;
        pBasler->ExposureAuto.SetValue(Basler_UniversalCameraParams::ExposureAuto_Continuous);
        
        CHECK_ARW(pBasler->AutoTargetValue)
        pBasler->AutoTargetValue.SetValue(70);
        std::cout << "[BaslerAdapter::initCamera] Current autoexposure set to target value: " << pBasler->AutoTargetValue.GetValue() << std::endl;
        
        CHECK_ARW(pBasler->BalanceWhiteAuto)
        std::cout << "[BaslerAdapter::initCamera] Auto Balance White enabled in continuous mode." << std::endl;
        pBasler->BalanceWhiteAuto.SetValue(Basler_UniversalCameraParams::BalanceWhiteAuto_Continuous);

        std::cout << "[BaslerAdapter::initCamera] Frame Rate should be handled with loop that calls trigger as no continuous capture is enabled." << std::endl;
        
        // No PTP in this camera :)
        bool b = pBasler->GevSupportedIEEE1588.GetValue();
        std::cout << "[BaslerAdapter::initCamera] Is PTP supported by " << getName() << "? " << (b ? "Yes" : "No") << std::endl;

        GenApi::CIntegerPtr tsFreqNode(pBasler->GetNodeMap().GetNode("GevTimestampTickFrequency"));
        CHECK_AR(tsFreqNode)
        tick_frequency = tsFreqNode->GetValue();
        std::cout << "[BaslerAdapter::initCamera] Timestamp Tick Frequency: " << tick_frequency << " ticks/s" << std::endl;
        
        ////////////////////////////////////
        //  Metadata extraction enabling  //
        ////////////////////////////////////

        CHECK_ARW(pBasler->ChunkModeActive);
        pBasler->ChunkModeActive.SetValue(true);

        pBasler->ChunkSelector.SetValue("Framecounter");
        pBasler->ChunkEnable.SetValue(true);

        pBasler->ChunkSelector.SetValue("ExposureTime");
        pBasler->ChunkEnable.SetValue(true);

        pBasler->ChunkSelector.SetValue("GainAll");
        pBasler->ChunkEnable.SetValue(true);
        
        return true;
    }
    catch (const Pylon::GenericException &e)
    {
        std::cerr << "[BaslerAdapter::initCamera] Pylon exception: " << e.GetDescription() << std::endl;
        return false;
    }
}

/**
 * @brief Function that handle acquisition init. Note that it has to start after all configuration is set.
 * @return true or false depending on image acquisition result
 */
bool beginAcquisition()
{
    CHECK_POINTER(pBasler);
    if (!pBasler->IsGrabbing())
    {
        std::cout << "[BaslerAdapter::beginAcquisition] Begin acquisition." << std::endl;
        pBasler->StartGrabbing(Pylon::GrabStrategy_LatestImageOnly);
    }
    else
    {
        std::cout << "[BaslerAdapter::beginAcquisition] Acquisition already started." << std::endl;
    }

    // Timestamp calibration initialization :)
    g_calibration = calibrateTimestamps(30);
    return true;
}

/**
 * @brief Function that handle acquisition end.
 */
bool endAcquisition()
{
    CHECK_POINTER(pBasler);
    if (pBasler->IsGrabbing())
    {
        std::cout << "[BaslerAdapter::endAcquisition] End acquisition." << std::endl;
        
        // Mostrar estadísticas finales de calibración
        if (g_calibration.initialized && g_calibration.samples_count > 0) {
            std::cout << "[Calibration] Estadísticas finales:" << std::endl;
            std::cout << "              - Muestras procesadas: " << g_calibration.samples_count << std::endl;
            std::cout << "              - Offset final: " << g_calibration.offset_ns/1e6 << " ms" << std::endl;
            std::cout << "              - Error medio: " << g_calibration.getMeanError() << " ms" << std::endl;
            std::cout << "              - Desv. estándar: " << g_calibration.getStdDevError() << " ms" << std::endl;
        }
        
        pBasler->StopGrabbing();
    }
    else
    {
        std::cout << "[BaslerAdapter::endAcquisition] Acquisition is not running." << std::endl;
    }
    return true;
}

/**
 * @brief Configure camera as Master to be synchronized through hardware trigger
 * @return true or false depending on image acquisition
 */
bool setAsMaster()
{
    CHECK_POINTER(pBasler);
    std::cout << "No master configuration for now :)" << std::endl;
    return true;

    // Setup continuous acquisition as trigger
    pBasler->AcquisitionMode.SetValue(Basler_UniversalCameraParams::AcquisitionMode_SingleFrame);

    pBasler->TriggerSelector.SetValue("FrameStart");
    pBasler->TriggerMode.SetValue("On");
    pBasler->TriggerSource.SetValue("Software");

    try
    {
        // Select Line 2 (output line)
        CHECK_AW(pBasler->LineSelector);
        pBasler->LineSelector.SetValue(Basler_UniversalCameraParams::LineSelector_Out1);

        // Set it as output
        CHECK_AW(pBasler->LineMode);
        pBasler->LineMode.SetValue(Basler_UniversalCameraParams::LineMode_Output);

        // Set the source signal to User Output 1
        CHECK_AW(pBasler->LineSource);
        pBasler->LineSource.SetValue(Basler_UniversalCameraParams::LineSource_ExposureActive);
    }
    catch (const Pylon::GenericException &e)
    {
        std::cerr << "[BaslerAdapter::setAsMaster] Pylon exception: " << e.GetDescription() << std::endl;
        return false;
    }

    std::cout << "[BaslerAdapter::setAsMaster] Configured internal trigger and signal output." << std::endl;

    return true;   
}

/**
 * @brief Configure camera as Slave to be synchronized through hardware trigger
 * @return true or false depending on image acquisition
 */
bool setAsSlave()
{
    CHECK_POINTER(pBasler);
    std::cerr << "[BaslerAdapter::setAsSlave] ************************************" << std::endl;
    std::cerr << "[BaslerAdapter::setAsSlave] * EMPTY FUNCTION. NOT IMPLEMENTED. *" << std::endl;
    std::cerr << "[BaslerAdapter::setAsSlave] *  Calling setAsMaster() function  *" << std::endl;
    std::cerr << "[BaslerAdapter::setAsSlave] ************************************" << std::endl;
    bool result = setAsMaster();
    return result;   
}

/**
 * @brief Function that handles image acquisition. Returns image in CV format.
 * @param image CV mat reference to be filled with image
 * @return true or false depending on image acquisition
 */
bool acquireImage(cv::Mat& image, ImageMetadata& metadata)
{
    CHECK_POINTER(pBasler);
    try
    {
        // This smart pointer will receive the grab result data.
        Pylon::CGrabResultPtr ptrGrabResult;
        cv::Mat openCvImage;
        
        Pylon::CImageFormatConverter formatConverter;
        formatConverter.OutputPixelFormat = Pylon::PixelType_BGR8packed;
        Pylon::CPylonImage pylonImage;

        // Capturar timestamp del PC antes y después del trigger
        auto pc_start = std::chrono::system_clock::now();
        
        // Wait for an image and then retrieve it. A timeout of 1000 ms is used.
        metadata.initTimestamps(); //Stores timetag when requested to avoid communication delay difference between cameras
        pBasler->ExecuteSoftwareTrigger();
        metadata.triggerAck();
        pBasler->RetrieveResult( 1000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);
        
        auto pc_end = std::chrono::system_clock::now();
        
        // Image grabbed successfully?
        if (!ptrGrabResult)
        {
            std::cout << "[BaslerAdapter::acquireImage] No grab result reference." << std::endl;
            return false;
        }
        if (ptrGrabResult->GrabSucceeded())
        {
            // Access the image data.
            formatConverter.Convert(pylonImage, ptrGrabResult);
            // needs to be cloned so to not keep pointing to local raw data that will be destroyed after function finishes
            image = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t *)pylonImage.GetBuffer()).clone();
            
            // Obtener timestamp de la cámara
            auto camera_timestamp_ticks = ptrGrabResult->GetTimeStamp();
            int64_t cam_ns = camera_timestamp_ticks * 1e9 / tick_frequency;
            
            // Aplicar calibración actual al timestamp de la cámara
            metadata.camera_timestamp = g_calibration.offset_ns + cam_ns * g_calibration.slope;
            
            // Actualizar calibración con esta nueva muestra
            // Usar el promedio de pc_start y pc_end para mejor precisión
            int64_t pc_ns = (pc_start.time_since_epoch().count() + 
                           pc_end.time_since_epoch().count()) / 2;
            g_calibration.updateWithSample(camera_timestamp_ticks, pc_ns, tick_frequency);
            
            // std::cout << "[BaslerAdapter::acquireImage] Image acquired with timestamp (ns): " << metadata.camera_timestamp << std::endl;
            
            /*****************************************
            **   Extract metadata from chunk data   **
            ******************************************/
            metadata.width = ptrGrabResult->GetWidth();
            metadata.height = ptrGrabResult->GetHeight();
            
            GenApi::CEnumerationPtr pixelFormatNode(pBasler->GetNodeMap().GetNode("PixelFormat"));
            if (GenApi::IsReadable(pixelFormatNode))
            {
                std::string pixelFormatName = std::string(pixelFormatNode->GetCurrentEntry()->GetSymbolic());
                metadata.pixelFormat = pixelFormatName;
            }

            GenApi::INodeMap& chunkDataMap = ptrGrabResult->GetChunkDataNodeMap();
        
            // Framecounter
            GenApi::CIntegerPtr chunkFrameCounter(chunkDataMap.GetNode("ChunkFramecounter"));
            if (GenApi::IsReadable(chunkFrameCounter))
            {
                int64_t frameCounter = chunkFrameCounter->GetValue();
                metadata.frameCounter = frameCounter;
            }
        
            // ExposureTime
            GenApi::CFloatPtr chunkExposure(chunkDataMap.GetNode("ChunkExposureTime"));
            if (GenApi::IsReadable(chunkExposure))
            {
                double exposure_us = chunkExposure->GetValue();
                metadata.setExposure(static_cast<uint64_t>(exposure_us * 1000.0)); // store in nanoseconds
            }
        
            // GainAll
            GenApi::CFloatPtr chunkGain(chunkDataMap.GetNode("ChunkGainAll"));
            if (GenApi::IsReadable(chunkGain))
            {
                double gain = chunkGain->GetValue();
                metadata.gain = gain;
            }

            // Gain
            GenApi::CFloatPtr gainNode(pBasler->GetNodeMap().GetNode("Gain"));
            if (GenApi::IsReadable(gainNode))
            {
                double gain = gainNode->GetValue();
                metadata.gain = gain;
            }
        }
        else
        {
            std::cout << "[BaslerAdapter::acquireImage] Error: " << ptrGrabResult->GetErrorCode() << " " << ptrGrabResult->GetErrorDescription() << std::endl;
            return false;
        }
    }
    catch (const Pylon::GenericException &e)
    {
        std::cerr << "[BaslerAdapter::acquireImage] Pylon exception: " << e.GetDescription() << std::endl;
        return false;
    }
    return true;   
}

/**
 * @brief Function that handle all camera de-initialization, port closing and Pylon clean finishing.
 * @return true or false depending on image acquisition
 */
bool closeCamera()
{
    std::cout << "[BaslerAdapter::closeCamera] Close camera requested." << std::endl;
    // Deinitialize Basler
    if (pBasler)
    {
        endAcquisition();
        pBasler->Close();
        pBasler.reset();
    }

    // Releases all pylon resources. 
    Pylon::PylonTerminate(); 

    return true;
}