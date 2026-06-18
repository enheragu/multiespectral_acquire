/**
    * This file is a wrapper of Flir API for GeniCam camera acA 1600-60gc, exposes a common API to be wrapped into a
    * ROS node.
    */

#include <thread>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

// FLIR API
#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"

#include "camera_adapter.h"
#include "core/utils/image_metadata.h"
#include "core/utils/timestamp_calibration.h"


// Check if available and readable and writable; and combinations of them
#define CHECK_A(node){if (!IsAvailable(node)) \
{ \
    std::cout << "[FlirAdapter::" << __func__ << "] " << #node << " is not available." << std::endl; \
    return false; \
}}
#define CHECK_R(node){if (!IsReadable(node)) \
{ \
    std::cout << "[FlirAdapter::" << __func__ << "] " << #node << " is not readable." << std::endl; \
    return false; \
}}
#define CHECK_W(node){if (!IsWritable(node)) \
{ \
    std::cout << "[FlirAdapter::" << __func__ << "] " << #node << " is not writable." << std::endl; \
    return false; \
}}
#define CHECK_AR(node) {CHECK_A(node); CHECK_R(node); }
#define CHECK_AW(node) {CHECK_A(node); CHECK_W(node); }
#define CHECK_ARW(node) {CHECK_A(node); CHECK_R(node); CHECK_W(node); }

#define CHECK_POINTER(pointer){if (!pointer) \
{ \
    std::cout << "[FlirAdapter::" << __func__ << "] " << #pointer << " is not available." << std::endl; \
    return false; \
}}

// Reference to Flir camera to be handled
Spinnaker::CameraPtr pFlir = nullptr;
Spinnaker::CameraList flirCamList;
Spinnaker::SystemPtr flir_system;
std::string camera_name = "Default:FlirA68";  // Display name (usually ROS node name)
std::string model_name = "Unknown";  // Camera model name
double camera_exposure_time_ns = 33333333; // (30Hz in nanoseconds)
bool ptp_supported = false;
bool force_disable_ptp = false;  // Set true to force manual calibration even if PTP available

// Correction added to camera timestamps when PTP is active. Decided empirically
// in beginAcquisition(): 0 if the camera clock is already UTC, -37 s if it runs
// on the raw PTP/TAI timescale (matches ptp4l utc_offset=37 and the ouster
// driver's ptp_utc_tai_offset=-37).
int64_t g_ptp_correction_ns = 0;
constexpr int64_t kUtcTaiOffsetNs = 37LL * 1000000000LL;

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
    if(camera_name == "Default:FlirA68" && model_name != "Unknown")
    {
        return std::string("FLIR ") + model_name;
    }
    return camera_name;
}

/**
 * @brief Get the camera model name
 */
std::string getModelName()
{
    if(pFlir && pFlir->IsValid())
    {
        try {
            Spinnaker::GenApi::INodeMap& nodeMap = pFlir->GetTLDeviceNodeMap();
            Spinnaker::GenApi::CStringPtr ptrModelName = nodeMap.GetNode("DeviceModelName");
            if (IsAvailable(ptrModelName) && IsReadable(ptrModelName))
            {
                model_name = std::string("Flir ") + std::string(ptrModelName->GetValue());
            }
        } catch (...) {
            // Ignore errors
        }
    }
    return model_name;
}

/**
    * @brief Return image type to store it correctly
    */
std::string getType()
{
    return "lwir";
}

/**
 * @brief Calibrate FLIR timestamps using linear regression
 */
TimestampCalibration calibrateTimestamps(int num_samples = 30) {
    std::vector<int64_t> camera_times, pc_times;
    
    std::cout << "[FlirAdapter::calibrateTimestamps] Waiting for camera stabilization (2 seconds)..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    std::cout << "[FlirAdapter::calibrateTimestamps] Capturing " << num_samples << " samples for calibration..." << std::endl;
    
    for(int i = 0; i < num_samples; i++) {
        try {
            auto pc_start = std::chrono::system_clock::now();
            
            // Trigger + capture
            pFlir->TriggerSoftware.Execute();
            Spinnaker::ImagePtr pResultImage = pFlir->GetNextImage(5000);
            
            if(!pResultImage->IsIncomplete()) {
                int64_t cam_ns = pResultImage->GetTimeStamp();
                auto pc_end = std::chrono::system_clock::now();
                
                // Average PC time (reduce jitter)
                int64_t pc_ns = (pc_start.time_since_epoch().count() + 
                               pc_end.time_since_epoch().count()) / 2;
                
                camera_times.push_back(cam_ns);
                pc_times.push_back(pc_ns);
            }
            
            pResultImage->Release();
        } catch (Spinnaker::Exception& e) {
            std::cerr << "[FlirAdapter::calibrateTimestamps] Capture failed at sample " << i << ": " << e.what() << std::endl;
        }
        
        // Space out samples (except the last one)
        if(i < num_samples - 1) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    
    if(camera_times.size() < 3) {
        std::cerr << "[FlirAdapter::calibrateTimestamps] ERROR: Not enough valid samples (" 
                  << camera_times.size() << "). Using default calibration." << std::endl;
        TimestampCalibration default_cal;
        default_cal.slope = 1.0;
        default_cal.offset_ns = 0;
        default_cal.initialized = true;
        return default_cal;
    }
    
    // Use performInitialCalibration from utils (FLIR timestamps are already in nanoseconds, so tick_freq = 1e9)
    return performInitialCalibration(camera_times, pc_times, 1000000000LL, getName());
}

/**
    * @brief Helper function to convert IP Address to dotted format
    *        extracted from Pylon examplaes
    */
std::string GetDottedAddress(int64_t value)
{
    unsigned int inputValue = static_cast<unsigned int>(value);
    std::ostringstream convertValue;
    convertValue << ((inputValue & 0xFF000000) >> 24);
    convertValue << ".";
    convertValue << ((inputValue & 0x00FF0000) >> 16);
    convertValue << ".";
    convertValue << ((inputValue & 0x0000FF00) >> 8);
    convertValue << ".";
    convertValue << (inputValue & 0x000000FF);
    return convertValue.str().c_str();
}



/**
    * @brief This function configures GigE cameras discovered to an IP configuration that will work with Spinnaker.
    *        Configures from specific interface and camera index.
    * @param interface_n Interface index to configure.
    */
bool AutoConfigureFlirCamera(int interface_n = 1, int camera_index = 0)
{
    Spinnaker::SystemPtr pSystem = Spinnaker::System::GetInstance();
    std::cout << "[FlirAdapter::AutoConfigure] Setting all GigE cameras discovered from interface " << interface_n << " to an IP configuration that will work with Spinnaker:" << std::endl;

    Spinnaker::InterfaceList interfaceList = pSystem->GetInterfaces();
    // unsigned int interfaceCount = interfaceList.GetSize();
    { // otro ambito para estas variables
        Spinnaker::InterfacePtr pInterface = interfaceList.GetByIndex(interface_n);
        Spinnaker::GenApi::INodeMap& nodeMapInterface = pInterface->GetTLNodeMap();
        Spinnaker::GenApi::CEnumerationPtr ptrInterfaceType = nodeMapInterface.GetNode("InterfaceType");
        CHECK_AR(ptrInterfaceType);

        if (ptrInterfaceType->GetIntValue() != Spinnaker::InterfaceType_GigEVision)
        {
            std::cout << "[FlirAdapter::AutoConfigure] Interface is not a GigE Vision interface. Only forces on GigE interfaces sorryn't." << std::endl;
            return false; // Only force IP on GEV interface
        }
        if(pInterface->GetCameras().GetSize() == 0) 
        {
            std::cout << "[FlirAdapter::AutoConfigure] Force IP node not available for this interface (e.g. No Gige Cameras are connected to this interface)." << std::endl;
            return false;
        }

        Spinnaker::GenApi::CStringPtr ptrInterfaceDisplayName = nodeMapInterface.GetNode("InterfaceDisplayName");
        Spinnaker::GenICam::gcstring interfaceDisplayName = "Unknown Interface (Display name not readable)";
        CHECK_AR(ptrInterfaceDisplayName);
        std::cout << "[FlirAdapter::AutoConfigure] *** " << ptrInterfaceDisplayName->GetValue() << " ***" << std::endl;


        CHECK_AR(nodeMapInterface.GetNode("GevDeviceAutoForceIP"));
        CHECK_AR(pInterface->TLInterface.DeviceSelector.GetAccessMode());
        pInterface->TLInterface.DeviceSelector.SetValue(camera_index);
        pInterface->TLInterface.GevDeviceAutoForceIP.Execute();
        std::cout << "[FlirAdapter::AutoConfigure] AutoForceIP executed for camera at index " << camera_index << ", serial number: " << pInterface->TLInterface.DeviceSerialNumber.GetValue() << std::endl;
    }
    interfaceList.Clear();
    pSystem->ReleaseInstance();

    std::cout << "[FlirAdapter::AutoConfigure] Auto-configuration complete. Waiting for camera to be available again." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(3));
    return true;
}


/**
    * @brief Function that handle all Basler initializacion and configuration.
    * @return true or false depending on image acquisition
    */
bool initCamera(int frame_rate, std::string camera_ip)
{   
    // Reste cameras to a IP range compatible
    // AutoConfigureFlirCamera();
    std::cout << "[FlirAdapter::initCamera] Initialize Spinnaker system (frame_rate " << frame_rate << "; camera_ip " << camera_ip << ")." << std::endl;
    flir_system = Spinnaker::System::GetInstance();

    bool result = false;
    flirCamList = flir_system->GetCameras();
    if (flirCamList.GetSize()<=0)
    {
        std::cerr << "[FlirAdapter::initCamera] No cameras detected."  << std::endl;
        return false;
    }

    std::cerr << "[FlirAdapter::initCamera] "<<flirCamList.GetSize()<<" cameras detected."  << std::endl;
        
    // for (unsigned int i = 0; i < flirCamList.GetSize(); i++)
    // {
    //     try
    //     {
    //         pFlir = flirCamList.GetByIndex(i);
    //         pFlir->Init();
    //         break;
    //     }
    //     catch (Spinnaker::Exception& e)
    //     {
    //         // Error handling.
    //         std::cerr << "[FlirAdapter::initCamera] Coulnd not init camera with index: "<<i<<" " << e.what() << std::endl;
    //     }
    // }

    try
    {
        pFlir = flirCamList.GetBySerial("M0000726");
        CHECK_POINTER(pFlir);
        pFlir->Init();

        // Force-stop acquisition in case camera was left streaming by a crashed process.
        // IsStreaming() only reflects SDK state, not hardware state after SIGKILL,
        // so we unconditionally try EndAcquisition and ignore the exception if not streaming.
        // Then DeInit+Init to fully reset hardware state.
        try {
            pFlir->EndAcquisition();
            std::cout << "[FlirAdapter::initCamera] Camera was still streaming from previous session. Stopped." << std::endl;
        } catch (Spinnaker::Exception&) {
            // Not streaming — expected path on clean startup
        }
        pFlir->DeInit();
        pFlir->Init();

        // Ensure heartbeat is enabled with a short timeout.
        // Spinnaker debug mode can disable the heartbeat, causing the camera to stay
        // locked until power-cycled after a crash. With a 3 s timeout the camera
        // releases the control channel within seconds whenever this process dies.
        try {
            Spinnaker::GenApi::INodeMap& tlMap = pFlir->GetTLDeviceNodeMap();
            // Known node names across Spinnaker versions and FLIR models:
            for (const char* name : {"GevGVCPHeartbeatDisable", "GevHeartbeatDisable"}) {
                Spinnaker::GenApi::CBooleanPtr n = tlMap.GetNode(name);
                if (Spinnaker::GenApi::IsWritable(n)) {
                    n->SetValue(false);
                    std::cout << "[FlirAdapter::initCamera] Heartbeat enabled (" << name << ")." << std::endl;
                    break;
                }
            }
            Spinnaker::GenApi::CIntegerPtr pTimeout = tlMap.GetNode("GevHeartbeatTimeout");
            if (Spinnaker::GenApi::IsWritable(pTimeout)) {
                pTimeout->SetValue(3000);
                std::cout << "[FlirAdapter::initCamera] Heartbeat timeout set to 3000 ms." << std::endl;
            }
        } catch (const Spinnaker::Exception& e) {
            std::cerr << "[FlirAdapter::initCamera] Warning: could not configure heartbeat: "
                      << e.what() << " (non-fatal)" << std::endl;
        }

        // Continuous Acquisition
        // CHECK_ARW(pFlir->AcquisitionMode);
        // pFlir->AcquisitionMode.SetValue(Spinnaker::AcquisitionMode_Continuous);

        Spinnaker::GenApi::INodeMap& nodeMap = pFlir->GetNodeMap();
        Spinnaker::GenApi::CEnumerationPtr ptrAcquisitionMode = nodeMap.GetNode("AcquisitionMode");
        CHECK_ARW(ptrAcquisitionMode);

        Spinnaker::GenApi::CEnumEntryPtr  ptrAcquisitionModeContinuous = ptrAcquisitionMode->GetEntryByName("Continuous");
        CHECK_AR(ptrAcquisitionModeContinuous);
        const int64_t acquisitionModeContinuous = ptrAcquisitionModeContinuous->GetValue();
        ptrAcquisitionMode->SetIntValue(acquisitionModeContinuous);
        std::cout << "[FlirAdapter::initCamera] Acquisition mode set to Continuous..." << std::endl;

        // Spinnaker::GenApi::CEnumEntryPtr ptrAcquisitionModeSingleFrame = ptrAcquisitionMode->GetEntryByName("SingleFrame");
        // CHECK_AR(ptrAcquisitionModeSingleFrame);
        // const int64_t acquisitionModeSingleFrame = ptrAcquisitionModeSingleFrame->GetValue();
        // ptrAcquisitionMode->SetIntValue(acquisitionModeSingleFrame);  // SingleFrame = On-Demand
        // std::cout << "[FlirAdapter::initCamera] Acquisition mode set to single frame..." << std::endl;

        // Software Trigger
        Spinnaker::GenApi::CEnumerationPtr ptrTriggerMode = nodeMap.GetNode("TriggerMode");
        CHECK_AW(ptrTriggerMode); 
        Spinnaker::GenApi::CEnumEntryPtr ptrTriggerModeOn = ptrTriggerMode->GetEntryByName("On"); 
        ptrTriggerMode->SetIntValue(ptrTriggerModeOn->GetValue());

        Spinnaker::GenApi::CEnumerationPtr ptrTriggerSource = nodeMap.GetNode("TriggerSource");
        CHECK_AW(ptrTriggerSource);
        Spinnaker::GenApi::CEnumEntryPtr ptrTriggerSourceSoftware = ptrTriggerSource->GetEntryByName("Software"); 
        CHECK_AR(ptrTriggerSource);
        ptrTriggerSource->SetIntValue(ptrTriggerSourceSoftware->GetValue());
        
        std::cout << "[FlirAdapter::initCamera] Trigger Mode: " << pFlir->TriggerMode.GetCurrentEntry()->GetSymbolic() << std::endl;
        std::cout << "[FlirAdapter::initCamera] Trigger Source: " << pFlir->TriggerSource.GetCurrentEntry()->GetSymbolic() << std::endl;
        

        // Buffer of images can get full, takes newest first each time
        Spinnaker::GenApi::INodeMap& sNodeMap = pFlir->GetTLStreamNodeMap();
        Spinnaker::GenApi::CEnumerationPtr ptrHandlingMode = sNodeMap.GetNode("StreamBufferHandlingMode");
        CHECK_ARW(ptrHandlingMode);
        Spinnaker::GenApi::CEnumEntryPtr ptrHandlingModeEntry = ptrHandlingMode->GetEntryByName("NewestOnly");
        ptrHandlingMode->SetIntValue(ptrHandlingModeEntry->GetValue());


        // Enable PTP
        try {
            if (force_disable_ptp) {
                std::cout << "[FlirAdapter::initCamera] PTP force disabled by parameter. Will use manual timestamp calibration." << std::endl;
                ptp_supported = false;
            } else {
                Spinnaker::GenApi::CEnumerationPtr ptrPtpMode = nodeMap.GetNode("ptpMode");
                CHECK_AW(ptrPtpMode)
                Spinnaker::GenApi::CEnumEntryPtr ptrPtpModeAutomatic = ptrPtpMode->GetEntryByName("Automatic");
                ptrPtpMode->SetIntValue(ptrPtpModeAutomatic->GetValue());
                std::cout << "[FlirAdapter::initCamera] PTP Mode: Automatic" << std::endl;

                // Trust PTP only with EVIDENCE of an actual lock. The previous
                // code set ptp_supported=true just because the node existed,
                // and an unlocked A68 free-runs on camera uptime — observed
                // 2026-06-12: 6 h in "Synchronizing" publishing uptime stamps.
                // The A68 only steps its clock when achieving lock (typically
                // when it boots with the grandmaster already present).
                ptp_supported = false;
                for (int i = 0; i < 10; ++i) {
                    try {
                        Spinnaker::GenApi::CCommandPtr latch = nodeMap.GetNode("ptpDataSetLatch");
                        if (latch.IsValid() && Spinnaker::GenApi::IsWritable(latch)) latch->Execute();
                        Spinnaker::GenApi::CEnumerationPtr servo = nodeMap.GetNode("ptpServoStatus");
                        if (servo.IsValid() && Spinnaker::GenApi::IsReadable(servo)) {
                            std::string st(servo->GetCurrentEntry()->GetSymbolic());
                            if (i == 0 || i == 9)
                                std::cout << "[FlirAdapter::initCamera] ptpServoStatus = " << st
                                          << " (poll " << i+1 << "/10)" << std::endl;
                            if (st == "Locked") { ptp_supported = true; break; }
                        } else {
                            break;  // no servo status node — cannot verify lock
                        }
                    } catch (...) { break; }
                    std::this_thread::sleep_for(std::chrono::seconds(2));
                }
                std::cout << "[FlirAdapter::initCamera] PTP lock "
                          << (ptp_supported ? "CONFIRMED — using hardware timestamps."
                                            : "NOT confirmed — falling back to software calibration. "
                                              "(A power cycle of the camera with the PTP master running usually locks it.)")
                          << std::endl;
            }
        }
        catch (Spinnaker::Exception& e) {
            std::cout << "[FlirAdapter::initCamera] Could not enable PTP: " << e.what()
                      << ". Will use manual timestamp calibration." << std::endl;
            ptp_supported = false;
        }

        // --- PTP diagnostic dump (read-only, runs regardless of use_ptp) ---
        // Logs what the camera itself reports about its PTP state so we can
        // see, with the ptp4l grandmaster running, whether the A68 acts as
        // Master / Listening / Slave and what offset it believes it has.
        // ptpDataSetLatch is the standard latch command to refresh the
        // readable PTP registers; it does not alter configuration.
        {
            try {
                Spinnaker::GenApi::CCommandPtr latch = nodeMap.GetNode("ptpDataSetLatch");
                if (latch.IsValid() && Spinnaker::GenApi::IsWritable(latch)) latch->Execute();
            } catch (...) {}
            const char* ptp_diag_nodes[] = {
                "ptpEnable", "ptpMode", "ptpStatus", "ptpServoStatus",
                "ptpOffsetFromMaster", "ptpMeanPropagationDelay",
                "ptpClockID", "ptpParentClockID", "ptpGrandmasterClockID",
                "PtpStatus", "PtpOffset", "PtpClockOperationMode"
            };
            for (const char* node_name : ptp_diag_nodes) {
                try {
                    Spinnaker::GenApi::CValuePtr v = nodeMap.GetNode(node_name);
                    if (v.IsValid() && Spinnaker::GenApi::IsReadable(v)) {
                        std::cout << "[FlirAdapter::initCamera][PTP-DIAG] " << node_name
                                  << " = " << v->ToString() << std::endl;
                    }
                } catch (...) { /* node absent on this model — ignore */ }
            }
        }

        // Get and log camera model information
        getModelName();  // This populates model_name
        std::cout << "[" << getName() << "] Camera model: " << model_name << std::endl;
        std::cout << "[" << getName() << "] PTP support: " << (ptp_supported ? "YES (hardware synchronized)" : "NO (using manual calibration)") << std::endl;
        
        result = true;

    }
    catch (Spinnaker::Exception& e)
    {
        // Error handling.
        std::cerr << "[FlirAdapter::initCamera] Spinnaker exception: " << e.what() << std::endl;
        return false;
    }
    return result;
}

/**
    * @brief Function that handle acquisition init. Note that it has to start after all configuration is set.
    * @return true or false depending on image acquisition result
    */
bool beginAcquisition()
{
    CHECK_POINTER(pFlir);

    if (pFlir->IsStreaming() == false)
    {
        std::cout << "[FlirAdapter::beginAcquisition] Starting acquisition." << std::endl;
        pFlir->BeginAcquisition();
    }
    else
    {
        std::cout << "[FlirAdapter::beginAcquisition] Acquisition already started." << std::endl;
    }
    
    // PTP timescale probe: even with a confirmed lock, the camera clock may be
    // raw PTP/TAI (+37 s vs UTC) or already UTC. Decide with one test frame
    // instead of assuming. Anything else = not actually synced → software cal.
    if (ptp_supported)
    {
        try {
            auto pc_before = std::chrono::system_clock::now();
            pFlir->TriggerSoftware.Execute();
            Spinnaker::ImagePtr probe = pFlir->GetNextImage(5000);
            if (probe && !probe->IsIncomplete()) {
                int64_t cam_ns = static_cast<int64_t>(probe->GetTimeStamp());
                int64_t pc_ns  = pc_before.time_since_epoch().count();
                int64_t diff   = cam_ns - pc_ns;
                if (std::llabs(diff) < 5LL * 1000000000LL) {
                    g_ptp_correction_ns = 0;
                    std::cout << "[FlirAdapter::beginAcquisition] PTP timescale: UTC (diff "
                              << diff / 1e6 << " ms). No correction." << std::endl;
                } else if (std::llabs(diff - kUtcTaiOffsetNs) < 5LL * 1000000000LL) {
                    g_ptp_correction_ns = -kUtcTaiOffsetNs;
                    std::cout << "[FlirAdapter::beginAcquisition] PTP timescale: TAI (diff "
                              << diff / 1e6 << " ms). Applying -37 s correction." << std::endl;
                } else {
                    std::cout << "[FlirAdapter::beginAcquisition] PTP claims lock but camera time is "
                              << diff / 1e9 << " s off UTC — NOT trusting it, using software calibration." << std::endl;
                    ptp_supported = false;
                }
            } else {
                std::cout << "[FlirAdapter::beginAcquisition] PTP probe frame failed — using software calibration." << std::endl;
                ptp_supported = false;
            }
            if (probe) probe->Release();
        } catch (Spinnaker::Exception& e) {
            std::cout << "[FlirAdapter::beginAcquisition] PTP probe exception: " << e.what()
                      << " — using software calibration." << std::endl;
            ptp_supported = false;
        }
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
    CHECK_POINTER(pFlir);
    if (pFlir->IsStreaming())
    {
        std::cout << "[FlirAdapter::endAcquisition] End acquisition." << std::endl;
        
        // Show final calibration statistics
        if (!ptp_supported && g_calibration.initialized) {
            g_calibration.printFinalStats();
        }
        
        pFlir->EndAcquisition();
    }
    else
    {
        std::cout << "[FlirAdapter::endAcquisition] Acquisition is not running." << std::endl;
    }
    return true;
}

/**
    * @brief Configure camera as Master to be synchronized through hardware trigger
    * @return true or false depending on image acquisition
    */
bool setAsMaster()
{
    
    CHECK_POINTER(pFlir);

    std::cout << "[FlirAdapter::setAsMaster] NO MASTER SETUP FOR NOW IN FLIR" << std::endl;
    return true;

    bool result = true;
    try
    {
        // Set Line Selector to appropriate line (only necessary for non-BFS/BFLy cameras)
        CHECK_AW(pFlir->LineSelector);    
        pFlir->LineSelector.SetIntValue(Spinnaker::TriggerSource_Line2); // Pin 6 of I/O connector
        std::cout << "[FlirAdapter::setAsMaster] Line Selector: " << pFlir->LineSelector.GetCurrentEntry()->GetSymbolic() << std::endl;

        // Set Line Mode to Output
        CHECK_AW(pFlir->LineMode);
        pFlir->LineMode.SetIntValue(Spinnaker::LineMode_Output);

        // Set Line Source to ExposureActive
        CHECK_AW(pFlir->LineSource);
        pFlir->LineSource.SetIntValue(Spinnaker::LineSource_ExposureActive);
        std::cout << "[FlirAdapter::setAsMaster] Line Source: " << pFlir->LineSource.GetCurrentEntry()->GetSymbolic() << std::endl;

    }
    catch (Spinnaker::Exception& e)
    {
        std::cerr << "[FlirAdapter::setAsMaster] Exception: " << e.what() << std::endl;
        result = false;
    }
    return result; 
}   


/**
    * @brief Configure camera as Slave to be synchronized through hardware trigger
    * @return true or false depending on image acquisition
    */
bool setAsSlave() 
{
    
    CHECK_POINTER(pFlir);

    // TBC is already set as slave by default?¿
    bool result = true;
    try
    {        
        // The trigger must be disabled in order to configure it again
        CHECK_AW(pFlir->TriggerMode);
        pFlir->TriggerMode.SetIntValue(Spinnaker::TriggerMode_On);

        std::cout << "[FlirAdapter::setAsSlave] Trigger Mode: " << pFlir->TriggerMode.GetCurrentEntry()->GetSymbolic() << std::endl;

        // The trigger source must be set to hardware or software while trigger mode is off.
        CHECK_AW(pFlir->TriggerSource);
        pFlir->TriggerSource.SetIntValue(Spinnaker::TriggerSource_Line1); // Pin 5 of I/O connector
        
        std::cout << "[FlirAdapter::setAsSlave] Trigger Source: " << pFlir->TriggerSource.GetCurrentEntry()->GetSymbolic() << std::endl;
        
        // Set Trigger Activation to Rising Edge -> Line 1 is always asserted on the rising edge.
        // CHECK_AW(pFlir->TriggerActivation);
        // pFlir->TriggerActivation.SetIntValue(Spinnaker::TriggerActivation_RisingEdge);
        // std::cout << "[FlirAdapter::setAsSlave] Trigger Activation: " << pFlir->TriggerActivation.GetCurrentEntry()->GetSymbolic() << std::endl;
    }
    catch (Spinnaker::Exception& e)
    {
        std::cerr << "[FlirAdapter::setAsSlave] Exception: " << e.what() << std::endl;
        result = false;
    }

    return result; 
}

/**
    * @brief Function that handles image acquisition. Returns image in CV format.
    * @param image CV mat reference to be filled with image
    * @return true or false depending on image acquisition
    */
bool acquireImage(cv::Mat& image, ImageMetadata& metadata)
{
    CHECK_POINTER(pFlir);
    bool result = true;

    Spinnaker::ImagePtr pResultImage = nullptr;
    try
    {
        // Capture timestamp before/after trigger for calibration
        auto pc_start = std::chrono::system_clock::now();
        
        pFlir->TriggerSoftware.Execute();
        pResultImage = pFlir->GetNextImage(5000);
        
        auto pc_end = std::chrono::system_clock::now();
        if (!pResultImage)
        {
            std::cout << "[FlirAdapter::acquireImage] No grab result reference." << std::endl;
            return false;
        }
        if (pResultImage->IsIncomplete())
        {
            std::cerr << "[FlirAdapter::acquireImage] Image incomplete with image status " << pResultImage->GetImageStatus() << std::endl;
            pResultImage->Release();
            return false;
        }
        else
        {
            Spinnaker::ImageProcessor processor;
            processor.SetColorProcessing(Spinnaker::SPINNAKER_COLOR_PROCESSING_ALGORITHM_HQ_LINEAR);
            Spinnaker::ImagePtr convertedImage = processor.Convert(pResultImage, Spinnaker::PixelFormat_Mono8);

            unsigned int rows = convertedImage->GetHeight();
            unsigned int cols = convertedImage->GetWidth();
            unsigned int num_channels = convertedImage->GetNumChannels();
            void *image_data = convertedImage->GetData();
            unsigned int stride = convertedImage->GetStride();
            image = cv::Mat(rows, cols, (num_channels == 3) ? CV_8UC3 : CV_8UC1, image_data, stride).clone();
            
            /**************************
            **   Extract metadata    **
            ***************************/
            // Get timestamp in nanoseconds (already in ns for FLIR)
            auto timestamp_nanoseconds = convertedImage->GetTimeStamp();
            metadata.camera_timestamp = static_cast<uint64_t>(
                static_cast<int64_t>(timestamp_nanoseconds) + g_ptp_correction_ns);
            metadata.updateTimetag();

            // Apply software calibration if PTP not available
            if (!ptp_supported) {
                int64_t pc_ns = (pc_start.time_since_epoch().count() +
                               pc_end.time_since_epoch().count()) / 2;
                metadata.camera_timestamp = g_calibration.offset_ns +
                                           timestamp_nanoseconds * g_calibration.slope;
                g_calibration.updateWithSample(timestamp_nanoseconds, pc_ns, 1000000000LL);
            }
            
            metadata.frameCounter = pResultImage->GetFrameID();
            metadata.width = pResultImage->GetWidth();
            metadata.height = pResultImage->GetHeight();
            
            Spinnaker::GenApi::CEnumerationPtr pixelFormatNode = pFlir->GetNodeMap().GetNode("PixelFormat");
            if (Spinnaker::GenApi::IsReadable(pixelFormatNode))
            {
                Spinnaker::GenApi::CEnumEntryPtr entry = pixelFormatNode->GetCurrentEntry();
                if (Spinnaker::GenApi::IsReadable(entry))
                {
                    std::string pixelFormatName = std::string(entry->GetSymbolic());
                    metadata.pixelFormat = pixelFormatName;
                }
            }

            Spinnaker::GenApi::INodeMap& nodemap = pFlir->GetNodeMap();

            // Exposure
            Spinnaker::GenApi::CFloatPtr exposureTime = nodemap.GetNode("ExposureTime");
            if (Spinnaker::GenApi::IsReadable(exposureTime))
                metadata.setExposure(static_cast<uint64_t>(exposureTime->GetValue()));
            else
                metadata.setExposure(camera_exposure_time_ns);

            // Gain
            Spinnaker::GenApi::CFloatPtr gain = nodemap.GetNode("Gain");
            if (Spinnaker::GenApi::IsReadable(gain))
                metadata.gain = gain->GetValue();
            
                result = true;

        }      
        pResultImage->Release();  
    }
    catch (Spinnaker::Exception& e)
    {
        if (pResultImage) {pResultImage->Release();}

        // Error handling.
        // Just empty buffer wont be considered an error
        if (std::string(e.what()).find(std::string("Failed waiting for EventData on NEW_BUFFER_DATA event")) != std::string::npos)
        {
            std::cout << "[FlirAdapter::acquireImage] Empty buffer, no image to acquire." << std::endl;
        }
        else
        {
            std::cerr << "[FlirAdapter::acquireImage] Spinnaker exception: " << e.what() << std::endl;
            return false;
        }
    }
    return result;   
}

/**
    * @brief Function that handle all camera de-initialization, port closing and Spinnaker clean finishing.
    * @return true or false depending on image acquisition
    */
bool closeCamera()
{
    std::cout << "[FlirAdapter::closeCamera] Close camera requested." << std::endl;
    if (pFlir) 
    { 
        // std::cout << "[FlirAdapter::closeCamera] EndAcquisition." << std::endl;
        endAcquisition();
        // std::cout << "[FlirAdapter::closeCamera] DeInit." << std::endl;
        pFlir->DeInit();
        // std::cout << "[FlirAdapter::closeCamera] Set to nullptr." << std::endl;
        pFlir = nullptr;
    }
    flirCamList.Clear();
    std::cout << "[FlirAdapter::closeCamera] Release system." << std::endl;
    flir_system->ReleaseInstance();
    return true;
}