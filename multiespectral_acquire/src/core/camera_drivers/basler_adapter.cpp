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
int64_t max_exposure_time = 200000; // in microseconds (200 ms default)

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

    // On failure return an UNINITIALIZED calibration (initialized=false) so the
    // caller can retry or abort init. The previous behaviour (slope=1, offset=0,
    // initialized=true) made the node publish camera-boot-relative timestamps;
    // since the Basler is the main sync trigger that poisoned every buffer
    // handler until the adaptive clock-step correction converged.

    std::cout << "[BaslerAdapter::"<<getName()<<"::calibrateTimestamps] Waiting for camera stabilization (2 seconds)..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(2));

    std::cout << "[BaslerAdapter::"<<getName()<<"::calibrateTimestamps] Capturing " << num_samples << " samples for calibration..." << std::endl;

    for(int i = 0; i < num_samples; i++) {
        try {
            auto pc_start = std::chrono::system_clock::now();

            pBasler->ExecuteSoftwareTrigger();
            Pylon::CGrabResultPtr ptrGrabResult;
            pBasler->RetrieveResult(1000, ptrGrabResult);

            if(ptrGrabResult && ptrGrabResult->GrabSucceeded()) {
                int64_t cam_ticks = ptrGrabResult->GetTimeStamp();
                // The camera latches its timestamp at exposure START (FrameStart
                // trigger), which happens right after ExecuteSoftwareTrigger().
                // pc_start is therefore the correct estimator. The old midpoint
                // (pc_start+pc_end)/2 was biased late by half of
                // exposure+readout+transfer — up to ~100 ms with auto-exposure
                // at its 200 ms limit, and variable with scene brightness.
                int64_t pc_ns = pc_start.time_since_epoch().count();
                camera_times.push_back(cam_ticks);
                pc_times.push_back(pc_ns);
            }
        }
        catch (const Pylon::GenericException &e) {
            std::cerr << "[BaslerAdapter::"<<getName()<<"::calibrateTimestamps] Pylon exception on sample "
                      << i << ": " << e.GetDescription() << ". Aborting calibration." << std::endl;
            return TimestampCalibration();
        }
        catch (...) {
            std::cerr << "[BaslerAdapter::"<<getName()<<"::calibrateTimestamps] Unknown exception on sample "
                      << i << ". Aborting calibration." << std::endl;
            return TimestampCalibration();
        }

        if(i < num_samples - 1) {
            std::this_thread::sleep_for(std::chrono::milliseconds(250));
        }
    }

    if(camera_times.size() < 3) {
        std::cerr << "[BaslerAdapter::"<<getName()<<"::calibrateTimestamps] ERROR: Not enough valid samples ("
                  << camera_times.size() << ")." << std::endl;
        return TimestampCalibration();
    }

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
        
        // Enumerate devices and find by IP.
        // Retry several times: the camera may not yet have responded to the GigE
        // Vision discovery broadcast (e.g. still booting, or briefly resetting after
        // a previous control-channel owner disconnected).
        Pylon::CTlFactory& tl_factory = Pylon::CTlFactory::GetInstance();
        Pylon::DeviceInfoList_t device_list;
        const int kEnumRetries = 5;
        const int kEnumRetryMs = 3000;
        for (int attempt = 0; attempt <= kEnumRetries; ++attempt)
        {
            device_list.clear();
            if (0 == tl_factory.EnumerateDevices(device_list))
            {
                if (attempt < kEnumRetries)
                {
                    std::cout << "[BaslerAdapter::"<<getName()<<"::initCamera] No cameras found, waiting "
                              << kEnumRetryMs/1000 << "s before retry ("
                              << attempt+1 << "/" << kEnumRetries << ")..." << std::endl;
                    std::this_thread::sleep_for(std::chrono::milliseconds(kEnumRetryMs));
                }
                else
                {
                    std::cerr << "[BaslerAdapter::"<<getName()<<"::initCamera] No available camera devices after "
                              << kEnumRetries << " attempts." << std::endl;
                    return false;
                }
            }
            else
            {
                break;
            }
        }

        // Find the device with the configured IP in the enumerated list.
        Pylon::DeviceInfoList_t::const_iterator target_dev = device_list.end();
        for (auto it = device_list.begin(); it != device_list.end(); ++it)
        {
            if (camera_ip == std::string(it->GetIpAddress()))
            {
                target_dev = it;
                break;
            }
        }
        if (target_dev == device_list.end())
        {
            std::cerr << "[BaslerAdapter::"<<getName()<<"::initCamera] Could not find camera with configured IP: " << camera_ip << std::endl;
            return false;
        }
        std::cout << "[BaslerAdapter::"<<getName()<<"::initCamera] Found camera device:"
                  << " Device Model: " << target_dev->GetModelName() << ";"
                  << " Device User Id: " << target_dev->GetUserDefinedName() << std::endl;

        // CreateDevice + constructor may download and parse the camera's GenICam XML.
        // ParseXmlBuffer can fail if the camera just power-cycled and its XML server
        // isn't ready. Retry up to 5× with 3 s gaps (same pattern as EnumerateDevices).
        const int kOpenRetries = 5;
        const int kOpenRetryMs = 3000;
        for (int attempt = 0; attempt <= kOpenRetries; ++attempt)
        {
            try
            {
                pBasler.reset(new Pylon::CBaslerUniversalInstantCamera(
                    tl_factory.CreateDevice(*target_dev)));
                pBasler->Open();
                break;   // success — XML parsed, device open
            }
            catch (Pylon::GenericException& e)
            {
                pBasler.reset();  // release before retry
                if (attempt < kOpenRetries)
                {
                    std::cerr << "[BaslerAdapter::"<<getName()<<"::initCamera] CreateDevice/Open failed ("
                              << e.GetDescription() << "), waiting "
                              << kOpenRetryMs/1000 << "s before retry ("
                              << attempt+1 << "/" << kOpenRetries << ")..." << std::endl;
                    std::this_thread::sleep_for(std::chrono::milliseconds(kOpenRetryMs));
                }
                else
                {
                    std::cerr << "[BaslerAdapter::"<<getName()<<"::initCamera] CreateDevice/Open failed after "
                              << kOpenRetries << " attempts: " << e.GetDescription() << std::endl;
                    return false;
                }
            }
        }
        if (!pBasler)
        {
            std::cerr << "[BaslerAdapter::"<<getName()<<"::initCamera] Camera object null after open." << std::endl;
            return false;
        }
        std::cerr << "[BaslerAdapter::"<<getName()<<"::initCamera] Camera opened: "
                  << pBasler->GetDeviceInfo().GetModelName() << std::endl;

        // Set heartbeat timeout to 10 s. Pylon's background heartbeat thread fires
        // every timeout/2 = 5 s. 3 s was too tight at first boot when CPU load is
        // high and the heartbeat thread could be delayed past the deadline, causing
        // the camera to drop the control channel after calibration. After a crash
        // the camera releases at most 10 s later; respawn_delay is set to 15 s so
        // the new process always opens a clean channel.
        // NOTE: do NOT raise this above respawn_delay (15 s) or the respawned
        // process will collide with the old GVCP session (0xE1018006).
        TRY_CONFIG(pBasler->GevHeartbeatTimeout)
        {
            pBasler->GevHeartbeatTimeout.SetValue(10000);
            std::cout << "[BaslerAdapter::"<<getName()<<"::initCamera] Heartbeat timeout set to 10000 ms." << std::endl;
        }

        // Explicitly lock the streaming packet size to 1500 bytes (standard Ethernet
        // MTU). Without this the camera keeps whatever was last written to its flash
        // (e.g. 8192 bytes from a PylonViewer/GigEConfigurator session). If the
        // Mokerlink switch cannot handle jumbo frames that silently drops all image
        // data. 1500 bytes is always safe; raise it only after confirming the switch
        // supports a larger MTU end-to-end.
        TRY_CONFIG(pBasler->GevSCPSPacketSize)
        {
            pBasler->GevSCPSPacketSize.SetValue(1500);
            std::cout << "[BaslerAdapter::"<<getName()<<"::initCamera] Streaming packet size set to 1500 bytes." << std::endl;
        }

        // Inter-packet delay (GevSCPD, in timestamp ticks: 8 ns at 125 MHz).
        // Without it each 1.9 MB frame bursts ~1300 packets at line rate; if the
        // FLIR (30 Hz) bursts at the same moment the Mokerlink switch must buffer
        // one of the streams and its shared buffer is far smaller than a frame,
        // so it tail-drops — including GVCP control packets, which is a plausible
        // cause of the recurring "Control channel not open" drops.
        // 4500 ticks = 36 us gap → ~250 Mbit/s effective → ~60 ms per frame,
        // irrelevant at the 1 Hz trigger rate.
        TRY_CONFIG(pBasler->GevSCPD)
        {
            pBasler->GevSCPD.SetValue(4500);
            std::cout << "[BaslerAdapter::"<<getName()<<"::initCamera] Inter-packet delay (GevSCPD) set to 4500 ticks (~36 us)." << std::endl;
        }

        // Store model name for logging
        model_name = std::string(pBasler->GetDeviceInfo().GetModelName());

        // Enable Auto Exposure (set to Continuous mode) with upper limit to avoid movement blurr
        TRY_CONFIG(pBasler->AutoExposureTimeAbsUpperLimit)
        {
            pBasler->AutoExposureTimeAbsUpperLimit.SetValue(max_exposure_time);
            std::cout << "[BaslerAdapter::"<<getName()<<"::initCamera] Auto exposure max limit set to: " << max_exposure_time << " microseconds." << std::endl;
        }

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

        TRY_CONFIG(pBasler->GainAuto)
        {
            pBasler->GainAuto.SetValue(Basler_UniversalCameraParams::GainAuto_Continuous);
            std::cout << "[BaslerAdapter::"<<getName()<<"::initCamera] Auto gain enabled in continuous mode." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
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

        if (IsAvailable(pBasler->ChunkModeActive) && IsWritable(pBasler->ChunkModeActive))
        {
            pBasler->ChunkModeActive.SetValue(true);
            std::cout << "[BaslerAdapter::"<<getName()<<"::initCamera] Chunk mode enabled for metadata extraction." << std::endl;
            
            std::vector<std::string> chunk_names = {"Gain", "GainAll", "ExposureTime", "Framecounter"};
            for (const auto& name : chunk_names)
            {
                try {
                    pBasler->ChunkSelector.SetValue(name.c_str());
                    pBasler->ChunkEnable.SetValue(true);
                    if (pBasler->ChunkEnable.IsReadable() && pBasler->ChunkEnable.GetValue()) {
                        std::cout << "[BaslerAdapter::"<<getName()<<"::initCamera]  ✓ Chunk '" << name << "' habilitado" << std::endl;
                    }
                } catch (...) {
                    std::cout << "[BaslerAdapter::"<<getName()<<"::initCamera]  ✗ Chunk '" << name << "' no disponible" << std::endl;
                }
            }
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
    try
    {
        if (!pBasler->IsGrabbing())
        {
            std::cout << "[BaslerAdapter::"<<getName()<<"::beginAcquisition] Begin acquisition." << std::endl;
            pBasler->StartGrabbing(Pylon::GrabStrategy_LatestImageOnly);
        }
        else
        {
            std::cout << "[BaslerAdapter::"<<getName()<<"::beginAcquisition] Acquisition already started." << std::endl;
        }
    }
    catch (const Pylon::GenericException &e)
    {
        std::cerr << "[BaslerAdapter::"<<getName()<<"::beginAcquisition] Pylon exception: " << e.GetDescription() << std::endl;
        return false;
    }
    catch (...)
    {
        std::cerr << "[BaslerAdapter::"<<getName()<<"::beginAcquisition] Unknown exception starting grab." << std::endl;
        return false;
    }

    // Software timestamp calibration if PTP not available.
    // Retry in-process a few times (the camera may still be settling after a
    // reboot/respawn). If it never succeeds, FAIL init so the node respawns
    // cleanly instead of publishing garbage timestamps as the main trigger.
    if (!ptp_supported)
    {
        const int kCalibAttempts = 3;
        for (int attempt = 1; attempt <= kCalibAttempts; ++attempt)
        {
            g_calibration = calibrateTimestamps(30);
            if (g_calibration.initialized) break;
            std::cerr << "[BaslerAdapter::"<<getName()<<"::beginAcquisition] Timestamp calibration failed (attempt "
                      << attempt << "/" << kCalibAttempts << ")." << std::endl;
            if (attempt < kCalibAttempts)
                std::this_thread::sleep_for(std::chrono::seconds(5));
        }
        if (!g_calibration.initialized)
        {
            std::cerr << "[BaslerAdapter::"<<getName()<<"::beginAcquisition] Timestamp calibration failed after "
                      << kCalibAttempts << " attempts — failing init (node will respawn)." << std::endl;
            return false;
        }
    }
    return true;
}

/**
 * @brief Function that handle acquisition end.
 */
bool endAcquisition()
{
    CHECK_POINTER(pBasler);
    try
    {
        if (pBasler->IsGrabbing())
        {
            std::cout << "[BaslerAdapter::"<<getName()<<"::endAcquisition] End acquisition." << std::endl;

            if (!ptp_supported && g_calibration.initialized) {
                g_calibration.printFinalStats();
            }

            pBasler->StopGrabbing();
        }
        else
        {
            std::cout << "[BaslerAdapter::"<<getName()<<"::endAcquisition] Acquisition is not running." << std::endl;
        }
    }
    catch (const Pylon::GenericException &e)
    {
        std::cerr << "[BaslerAdapter::"<<getName()<<"::endAcquisition] Pylon exception: " << e.GetDescription() << std::endl;
        return false;
    }
    catch (...)
    {
        std::cerr << "[BaslerAdapter::"<<getName()<<"::endAcquisition] Unknown exception stopping grab." << std::endl;
        return false;
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
            metadata.updateTimetag();

            // Apply software calibration if PTP not available
            if (!ptp_supported) {
                if (!g_calibration.initialized) {
                    // Should not happen (beginAcquisition fails without a valid
                    // calibration) — refuse to publish uncalibrated timestamps.
                    std::cerr << "[BaslerAdapter::"<<getName()<<"::acquireImage] No valid timestamp calibration." << std::endl;
                    return false;
                }
                // pc_start = exposure start estimator (see calibrateTimestamps).
                int64_t pc_ns = pc_start.time_since_epoch().count();
                metadata.camera_timestamp = g_calibration.offset_ns + cam_ns * g_calibration.slope;
                g_calibration.updateWithSample(camera_timestamp_ticks, pc_ns, tick_frequency);
            }
            
            // std::cout << "[BaslerAdapter::"<<getName()<<"::acquireImage] Image acquired with timestamp (ns): " << metadata.camera_timestamp << std::endl;
            
            /*****************************************
            **   Extract metadata from chunk data   **
            ******************************************/
            metadata.width = ptrGrabResult->GetWidth();
            metadata.height = ptrGrabResult->GetHeight();

            // PixelFormat is constant during a session — read it over GVCP only
            // once instead of one network round-trip per frame.
            static std::string cached_pixel_format;
            if (cached_pixel_format.empty())
            {
                GenApi::CEnumerationPtr pixelFormatNode(pBasler->GetNodeMap().GetNode("PixelFormat"));
                if (GenApi::IsReadable(pixelFormatNode))
                {
                    cached_pixel_format = std::string(pixelFormatNode->GetCurrentEntry()->GetSymbolic());
                }
            }
            if (!cached_pixel_format.empty())
            {
                metadata.pixelFormat = cached_pixel_format;
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

            // 1. Try chunk data (only if ChunkData payload)
            bool gain_read = false;
            if (ptrGrabResult->GetPayloadType() == Pylon::PayloadType_ChunkData) {
                try {
                    GenApi::INodeMap& chunkDataMap = ptrGrabResult->GetChunkDataNodeMap();
                    
                    GenApi::CFloatPtr chunkGain(chunkDataMap.GetNode("ChunkGain"));
                    if (!GenApi::IsReadable(chunkGain)) {
                        chunkGain = chunkDataMap.GetNode("ChunkGainAll");
                    }
                    if (GenApi::IsReadable(chunkGain)) {
                        metadata.gain = chunkGain->GetValue();
                        gain_read = true;
                    }
                }
                catch (...) {
                    // Chunk access failed, continue to fallback
                }
            }

            // 2. Fallback: direct camera parameters (multiple names)
            if (!gain_read || metadata.gain < 0) {
                std::vector<std::string> gain_names = {"Gain", "GainRaw", "GainAll"};
                for (const auto& name : gain_names) {
                    try {
                        GenApi::CFloatPtr gainNode(pBasler->GetNodeMap().GetNode(name.c_str()));
                        if (GenApi::IsReadable(gainNode)) {
                            metadata.gain = gainNode->GetValue();
                            gain_read = true;
                            break;
                        }
                    }
                    catch (...) {
                        // Try next name
                    }
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
    if (pBasler)
    {
        endAcquisition();
        try
        {
            pBasler->Close();
        }
        catch (const Pylon::GenericException &e)
        {
            std::cerr << "[BaslerAdapter::"<<getName()<<"::closeCamera] Pylon exception closing: " << e.GetDescription() << std::endl;
        }
        catch (...)
        {
            std::cerr << "[BaslerAdapter::"<<getName()<<"::closeCamera] Unknown exception closing camera." << std::endl;
        }
        pBasler.reset();
    }

    try
    {
        Pylon::PylonTerminate();
    }
    catch (...)
    {
        std::cerr << "[BaslerAdapter::"<<getName()<<"::closeCamera] Exception during PylonTerminate." << std::endl;
    }

    return true;
}