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
#include "core/utils/image_metadata.h"
#include "core/utils/timestamp_calibration.h"

// Check if available and readable and writable; and combinations of them
// These macros FAIL the function if the node is not available (for CRITICAL settings)
#define CHECK_A(node){if (!IsAvailable(node)) \
{ \
    std::cout << "[BaslerAdapter::"<<getName()<<"::"<< __func__ << "] " << #node << " is not available." << std::endl; \
    return false; \
}}
#define CHECK_R(node){if (!IsReadable(node)) \
{ \
    std::cout << "[BaslerAdapter::"<<getName()<<"::"<< __func__ << "] " << #node << " is not readable." << std::endl; \
    return false; \
}}
#define CHECK_W(node){if (!IsWritable(node)) \
{ \
    std::cout << "[BaslerAdapter::"<<getName()<<"::"<< __func__ << "] " << #node << " is not writable." << std::endl; \
    return false; \
}}
#define CHECK_AR(node) {CHECK_A(node); CHECK_R(node); }
#define CHECK_AW(node) {CHECK_A(node); CHECK_W(node); }
#define CHECK_ARW(node) {CHECK_A(node); CHECK_R(node); CHECK_W(node); }

// Optional checks - just warn and continue (for NON-CRITICAL settings)
#define TRY_CONFIG(node) \
    if (!IsAvailable(node)) { \
        std::cout << "[BaslerAdapter::"<<getName()<<"::"<< __func__ << "] " << #node << " not available (skipping)." << std::endl; \
    } else if (!IsWritable(node)) { \
        std::cout << "[BaslerAdapter::"<<getName()<<"::"<< __func__ << "] " << #node << " not writable (skipping)." << std::endl; \
    } else

#define CHECK_POINTER(pointer){if (!pointer) \
{ \
    std::cout << "[BaslerAdapter::"<<getName()<<"::"<< __func__ << "] " << #pointer << " is not available." << std::endl; \
    return false; \
}}

// Reference to basler camera to be handled
std::unique_ptr<Pylon::CBaslerUniversalInstantCamera> pBasler;
std::string camera_name = "Default:acA1600-60gc";  // Display name (usually ROS node name)
std::string model_name = "Unknown";  // Camera model name
int64_t tick_frequency = 1000000; // ticks per second default 1 MHz
bool ptp_supported = false;
bool force_disable_ptp = false;  // Set true to force manual calibration even if PTP available

// Global variable to store calibration
TimestampCalibration g_calibration;

/**
 * @brief Set the display name for the camera (typically the ROS node name)
 */
void setNodeName(const std::string& name)
{
    camera_name = name;
}

/**
 * @brief Get name of the camera for logging purposes (returns node name if set, model otherwise)
 */
std::string getName()
{
    // Update model name if camera available
    model_name = getModelName();
    
    // Return node name if set, otherwise model name
    if(camera_name == "Default:acA1600-60gc" && model_name != "Unknown")
    {
        return std::string("Basler ") + model_name;
    }
    return camera_name;
}

/**
 * @brief Get the camera model name
 */
std::string getModelName()
{
    if(pBasler)
    {
        model_name = std::string("Basler ") + std::string(pBasler->GetDeviceInfo().GetModelName());
    }
    return model_name;
}

/**
 * @brief Return image type to store it correctly
 */
std::string getType()
{
    return "visible";
}

TimestampCalibration calibrateTimestamps(int num_samples = 30) {
    std::vector<int64_t> camera_times, pc_times;
    
    std::cout << "[BaslerAdapter::"<<getName()<<"::calibrateTimestamps] Waiting for camera stabilization (2 seconds)..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    std::cout << "[BaslerAdapter::"<<getName()<<"::calibrateTimestamps] Capturing " << num_samples << " samples for calibration..." << std::endl;
    
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
        }
        
        // Space out samples (except the last one)
        if(i < num_samples - 1) {
            std::this_thread::sleep_for(std::chrono::milliseconds(250));
        }
    }
    
    if(camera_times.size() < 3) {
        std::cerr << "[BaslerAdapter::"<<getName()<<"::calibrateTimestamps] ERROR: Not enough valid samples (" 
                  << camera_times.size() << "). Using default calibration." << std::endl;
        TimestampCalibration default_cal;
        default_cal.slope = 1.0;
        default_cal.offset_ns = 0;
        default_cal.initialized = true;
        return default_cal;
    }
    
    // Use performInitialCalibration from utils
    return performInitialCalibration(camera_times, pc_times, tick_frequency, getName());
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
        std::cout << "[BaslerAdapter::"<<getName()<<"::initCamera] Initialize Pylon runtime for basler camera (frame_rate " << frame_rate << "; camera_ip " << camera_ip << ")." << std::endl;
        Pylon::PylonInitialize();
        
        // Enumerate devices and find by IP
        Pylon::CTlFactory& tl_factory = Pylon::CTlFactory::GetInstance();
        Pylon::DeviceInfoList_t device_list;
        
        if (0 == tl_factory.EnumerateDevices(device_list))
        {
            std::cerr << "[BaslerAdapter::"<<getName()<<"::initCamera] No available camera devices." << std::endl;
            return false;
        }
        else
        {
            bool found_desired_device = false;
            Pylon::DeviceInfoList_t::const_iterator it;
            for (it = device_list.begin(); it != device_list.end(); ++it)
            {
                std::string device_ip_found(it->GetIpAddress());
                if (0 == camera_ip.compare(device_ip_found))
                {
                    std::cout << "[BaslerAdapter::"<<getName()<<"::initCamera] Found camera device:"
                    << " Device Model: " << it->GetModelName() << "; "
                    << " with Device User Id: " << it->GetUserDefinedName() << std::endl;
                    
                    pBasler = std::unique_ptr<Pylon::CBaslerUniversalInstantCamera>(new Pylon::CBaslerUniversalInstantCamera(tl_factory.CreateDevice(*it)));
                    found_desired_device = true;
                    break;
                }
            }

            if (!found_desired_device)
            {
                std::cerr << "[BaslerAdapter::"<<getName()<<"::initCamera] Could not find camera with configured IP: " << camera_ip << std::endl;
                return false;
            }
        }

        if (!pBasler)
        {
            std::cerr << "[BaslerAdapter::"<<getName()<<"::initCamera] Camera with configured IP ("<<camera_ip<<") was not found."  << std::endl;
            return false;
        }
        else
        {
            std::cerr << "[BaslerAdapter::"<<getName()<<"::initCamera] Opening camera with: " << std::endl;
            std::cout << "\t\t· Model Name " << pBasler->GetDeviceInfo().GetModelName() << std::endl;
            std::cout << "\t\t· Friendly Name: " << pBasler->GetDeviceInfo().GetFriendlyName() << std::endl;
            std::cout << "\t\t· Current IP Addr: " << pBasler->GevCurrentIPAddress.ToStringOrDefault("<not readable>") << std::endl;
            std::cout << "\t\t· Requested IP: " << camera_ip << std::endl;
        }

        pBasler->Open();
        
        // Store model name for logging
        model_name = std::string(pBasler->GetDeviceInfo().GetModelName());

        // Enable Auto Exposure (set to Continuous mode)
        CHECK_ARW(pBasler->ExposureAuto)
        std::cout << "[BaslerAdapter::"<<getName()<<"::initCamera] Autoexposure enabled in continuous mode." << std::endl;
        pBasler->ExposureAuto.SetValue(Basler_UniversalCameraParams::ExposureAuto_Continuous);
        
        // Try to configure auto exposure target value (optional - not all cameras have this)
        TRY_CONFIG(pBasler->AutoTargetValue) {
            pBasler->AutoTargetValue.SetValue(70);
            std::cout << "[BaslerAdapter::"<<getName()<<"::initCamera] Autoexposure target value set to: " << pBasler->AutoTargetValue.GetValue() << std::endl;
        }
        
        // Try to configure auto balance white (optional)
        TRY_CONFIG(pBasler->BalanceWhiteAuto) {
            pBasler->BalanceWhiteAuto.SetValue(Basler_UniversalCameraParams::BalanceWhiteAuto_Continuous);
            std::cout << "[BaslerAdapter::"<<getName()<<"::initCamera] Auto Balance White enabled in continuous mode." << std::endl;
        }

        std::cout << "[BaslerAdapter::"<<getName()<<"::initCamera] Frame Rate should be handled with loop that calls trigger as no continuous capture is enabled." << std::endl;
        
        // Check if PTP is supported and enable it automatically
        try {
            if (force_disable_ptp) {
                std::cout << "[BaslerAdapter::"<<getName()<<"::initCamera] PTP force disabled by parameter. Will use manual timestamp calibration." << std::endl;
                ptp_supported = false;
            } else {
                // Check if PTP is supported (manual check with proper else handling)
                if (IsAvailable(pBasler->GevSupportedIEEE1588) && IsReadable(pBasler->GevSupportedIEEE1588))
                {
                    ptp_supported = pBasler->GevSupportedIEEE1588.GetValue();
                    std::cout << "[BaslerAdapter::"<<getName()<<"::initCamera] Is PTP supported by " << getName() << "? " << (ptp_supported ? "Yes" : "No") << std::endl;
                
                    if (ptp_supported)
                    {
                        // Try to enable PTP for automatic synchronization
                        if (IsAvailable(pBasler->GevIEEE1588) && IsWritable(pBasler->GevIEEE1588))
                        {
                            pBasler->GevIEEE1588.SetValue(true);
                            std::cout << "[BaslerAdapter::"<<getName()<<"::initCamera] PTP synchronization ENABLED. Camera will sync with PTP master automatically." << std::endl;
                            
                            // Give PTP some time to synchronize
                            std::cout << "[BaslerAdapter::"<<getName()<<"::initCamera] Waiting for PTP synchronization (2 seconds)..." << std::endl;
                            std::this_thread::sleep_for(std::chrono::seconds(2));
                        }
                        else
                        {
                            std::cout << "[BaslerAdapter::"<<getName()<<"::initCamera] PTP feature not writable. Will use manual timestamp calibration." << std::endl;
                            ptp_supported = false;
                        }
                    }
                    else
                    {
                        std::cout << "[BaslerAdapter::"<<getName()<<"::initCamera] PTP not supported by camera. Will use manual timestamp calibration." << std::endl;
                    }
                }
                else
                {
                    std::cout << "[BaslerAdapter::"<<getName()<<"::initCamera] Could not check PTP support (feature not available). Will use manual timestamp calibration." << std::endl;
                    ptp_supported = false;
                }
            }
        }
        catch (...) {
            std::cout << "[BaslerAdapter::"<<getName()<<"::initCamera] Exception checking PTP support. Will use manual timestamp calibration." << std::endl;
            ptp_supported = false;
        }

        GenApi::CIntegerPtr tsFreqNode(pBasler->GetNodeMap().GetNode("GevTimestampTickFrequency"));
        CHECK_AR(tsFreqNode)
        tick_frequency = tsFreqNode->GetValue();
        std::cout << "[" << getName() << "] Camera model: " << model_name << std::endl;
        std::cout << "[" << getName() << "] Timestamp Tick Frequency: " << tick_frequency << " ticks/s" << std::endl;
        std::cout << "[" << getName() << "] PTP support: " << (ptp_supported ? "YES (hardware synchronized)" : "NO (using manual calibration)") << std::endl;
        
        ////////////////////////////////////
        //  Metadata extraction enabling  //
        ////////////////////////////////////

        // Try to enable chunk mode for metadata extraction (optional - not all cameras support this)
        try {
            if (IsAvailable(pBasler->ChunkModeActive) && IsWritable(pBasler->ChunkModeActive)) {
                pBasler->ChunkModeActive.SetValue(true);
                std::cout << "[BaslerAdapter::"<<getName()<<"::initCamera] Chunk mode enabled for metadata extraction." << std::endl;

                // Try to enable specific chunks if available
                try {
                    pBasler->ChunkSelector.SetValue("Framecounter");
                    pBasler->ChunkEnable.SetValue(true);
                    std::cout << "[BaslerAdapter::"<<getName()<<"::initCamera] - Framecounter chunk enabled." << std::endl;
                } catch (const Pylon::GenericException &e) {
                    std::cout << "[BaslerAdapter::"<<getName()<<"::initCamera] - Framecounter chunk not available: " << e.GetDescription() << std::endl;
                }

                try {
                    pBasler->ChunkSelector.SetValue("ExposureTime");
                    pBasler->ChunkEnable.SetValue(true);
                    std::cout << "[BaslerAdapter::"<<getName()<<"::initCamera] - ExposureTime chunk enabled." << std::endl;
                } catch (const Pylon::GenericException &e) {
                    std::cout << "[BaslerAdapter::"<<getName()<<"::initCamera] - ExposureTime chunk not available: " << e.GetDescription() << std::endl;
                }

                try {
                    pBasler->ChunkSelector.SetValue("GainAll");
                    pBasler->ChunkEnable.SetValue(true);
                    std::cout << "[BaslerAdapter::"<<getName()<<"::initCamera] - GainAll chunk enabled." << std::endl;
                } catch (const Pylon::GenericException &e) {
                    std::cout << "[BaslerAdapter::"<<getName()<<"::initCamera] - GainAll chunk not available: " << e.GetDescription() << std::endl;
                }
            } else {
                std::cout << "[BaslerAdapter::"<<getName()<<"::initCamera] Chunk mode not available on this camera (will extract metadata from direct parameters)." << std::endl;
            }
        } catch (const Pylon::GenericException &e) {
            std::cout << "[BaslerAdapter::"<<getName()<<"::initCamera] Could not configure chunk mode: " << e.GetDescription() << " (not critical, continuing)." << std::endl;
        }
        
        return true;
    }
    catch (const Pylon::GenericException &e)
    {
        std::cerr << "[BaslerAdapter::"<<getName()<<"::initCamera] Pylon exception: " << e.GetDescription() << std::endl;
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
        std::cout << "[BaslerAdapter::"<<getName()<<"::beginAcquisition] Begin acquisition." << std::endl;
        pBasler->StartGrabbing(Pylon::GrabStrategy_LatestImageOnly);
    }
    else
    {
        std::cout << "[BaslerAdapter::"<<getName()<<"::beginAcquisition] Acquisition already started." << std::endl;
    }

    // Software timestamp calibration if PTP not available
    if (!ptp_supported)
    {
        g_calibration = calibrateTimestamps(30);
    }
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
        std::cout << "[BaslerAdapter::"<<getName()<<"::endAcquisition] End acquisition." << std::endl;
        
        // Show final calibration statistics
        if (!ptp_supported && g_calibration.initialized) {
            g_calibration.printFinalStats();
        }
        
        pBasler->StopGrabbing();
    }
    else
    {
        std::cout << "[BaslerAdapter::"<<getName()<<"::endAcquisition] Acquisition is not running." << std::endl;
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
        std::cerr << "[BaslerAdapter::"<<getName()<<"::setAsMaster] Pylon exception: " << e.GetDescription() << std::endl;
        return false;
    }

    std::cout << "[BaslerAdapter::"<<getName()<<"::setAsMaster] Configured internal trigger and signal output." << std::endl;

    return true;   
}

/**
 * @brief Configure camera as Slave to be synchronized through hardware trigger
 * @return true or false depending on image acquisition
 */
bool setAsSlave()
{
    CHECK_POINTER(pBasler);
    std::cerr << "[BaslerAdapter::"<<getName()<<"::setAsSlave] ************************************" << std::endl;
    std::cerr << "[BaslerAdapter::"<<getName()<<"::setAsSlave] * EMPTY FUNCTION. NOT IMPLEMENTED. *" << std::endl;
    std::cerr << "[BaslerAdapter::"<<getName()<<"::setAsSlave] *  Calling setAsMaster() function  *" << std::endl;
    std::cerr << "[BaslerAdapter::"<<getName()<<"::setAsSlave] ************************************" << std::endl;
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
        pBasler->ExecuteSoftwareTrigger();
        pBasler->RetrieveResult( 1000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);
        
        auto pc_end = std::chrono::system_clock::now();
        
        // Image grabbed successfully?
        if (!ptrGrabResult)
        {
            std::cout << "[BaslerAdapter::"<<getName()<<"::acquireImage] No grab result reference." << std::endl;
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
             
            metadata.camera_timestamp = cam_ns;
            
            // Apply software calibration if PTP not available
            if (!ptp_supported) {
                int64_t pc_ns = (pc_start.time_since_epoch().count() + 
                               pc_end.time_since_epoch().count()) / 2;
                metadata.camera_timestamp = g_calibration.offset_ns + cam_ns * g_calibration.slope;
                g_calibration.updateWithSample(camera_timestamp_ticks, pc_ns, tick_frequency);
            }
            
            // std::cout << "[BaslerAdapter::"<<getName()<<"::acquireImage] Image acquired with timestamp (ns): " << metadata.camera_timestamp << std::endl;
            
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
        
            // Framecounter (try chunk first, fallback to direct read)
            GenApi::CIntegerPtr chunkFrameCounter(chunkDataMap.GetNode("ChunkFramecounter"));
            if (GenApi::IsReadable(chunkFrameCounter))
            {
                int64_t frameCounter = chunkFrameCounter->GetValue();
                metadata.frameCounter = frameCounter;
            }
        
            // ExposureTime (try chunk first, fallback to direct parameter)
            bool exposure_read = false;
            GenApi::CFloatPtr chunkExposure(chunkDataMap.GetNode("ChunkExposureTime"));
            if (GenApi::IsReadable(chunkExposure))
            {
                double exposure_us = chunkExposure->GetValue();
                metadata.setExposure(static_cast<uint64_t>(exposure_us * 1000.0)); // store in nanoseconds
                exposure_read = true;
            }
            
            // Fallback: read ExposureTime directly from camera parameter
            if (!exposure_read)
            {
                GenApi::CFloatPtr exposureNode(pBasler->GetNodeMap().GetNode("ExposureTime"));
                if (GenApi::IsReadable(exposureNode))
                {
                    double exposure_us = exposureNode->GetValue();
                    metadata.setExposure(static_cast<uint64_t>(exposure_us * 1000.0)); // store in nanoseconds
                }
            }
        
            // GainAll (try chunk first, fallback to direct read)
            bool gain_read = false;
            GenApi::CFloatPtr chunkGain(chunkDataMap.GetNode("ChunkGainAll"));
            if (GenApi::IsReadable(chunkGain))
            {
                double gain = chunkGain->GetValue();
                metadata.gain = gain;
                gain_read = true;
            }

            // Fallback: read Gain directly from camera parameter
            if (!gain_read)
            {
                GenApi::CFloatPtr gainNode(pBasler->GetNodeMap().GetNode("Gain"));
                if (GenApi::IsReadable(gainNode))
                {
                    double gain = gainNode->GetValue();
                    metadata.gain = gain;
                }
            }
        }
        else
        {
            std::cout << "[BaslerAdapter::"<<getName()<<"::acquireImage] Error: " << ptrGrabResult->GetErrorCode() << " " << ptrGrabResult->GetErrorDescription() << std::endl;
            return false;
        }
    }
    catch (const Pylon::GenericException &e)
    {
        std::cerr << "[BaslerAdapter::"<<getName()<<"::acquireImage] Pylon exception: " << e.GetDescription() << std::endl;
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
    std::cout << "[BaslerAdapter::"<<getName()<<"::closeCamera] Close camera requested." << std::endl;
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