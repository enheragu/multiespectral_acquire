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

std::string FLIR_IP = "169.254.165.138";

/**
 * @brief Get name of the camera for logging purposes
 */
std::string getName()
{
    return "Flir A68";
}

/**
 * @brief Return image type to store it correctly
 */
std::string getType()
{
    return "lwir";
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
    unsigned int interfaceCount = interfaceList.GetSize();
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
bool initCamera(int frame_rate)
{   
    // Reste cameras to a IP range compatible
    // AutoConfigureFlirCamera();

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
        // pFlir = flirCamList.GetByIndex(0);
        pFlir = flirCamList.GetBySerial("M0000726");
        CHECK_POINTER(pFlir);
        pFlir->Init();

        // Continuous Acquisition
        // CHECK_ARW(pFlir->AcquisitionMode);
        // pFlir->AcquisitionMode.SetValue(Spinnaker::AcquisitionMode_Continuous);

        Spinnaker::GenApi::INodeMap& nodeMap = pFlir->GetNodeMap();
        Spinnaker::GenApi::CEnumerationPtr ptrAcquisitionMode = nodeMap.GetNode("AcquisitionMode");
        CHECK_ARW(ptrAcquisitionMode);
        Spinnaker::GenApi::CEnumEntryPtr ptrAcquisitionModeContinuous = ptrAcquisitionMode->GetEntryByName("Continuous");
        CHECK_AR(ptrAcquisitionModeContinuous);
        const int64_t acquisitionModeContinuous = ptrAcquisitionModeContinuous->GetValue();
        ptrAcquisitionMode->SetIntValue(acquisitionModeContinuous);

        std::cout << "[FlirAdapter::initCamera] Acquisition mode set to continuous..." << std::endl;


        // No autoexposure in A68!
        // Continuous auto exposure
        // CHECK_ARW(pFlir->ExposureAuto);
        // pFlir->ExposureAuto.SetValue(Spinnaker::ExposureAuto_Continuous);
     
        // FRAME RATE HAS TO BE CONFIGURED THROUGH TRIGGERING, FLIR CAMERA WILL WORK AS AN SLAVE
        // WITH FRAME RATE CONFIGURED IN MASTER CAMERA, BASLER

        // Buffer of images can get full, takes newest first each time
        Spinnaker::GenApi::INodeMap& sNodeMap = pFlir->GetTLStreamNodeMap();
        Spinnaker::GenApi::CEnumerationPtr ptrHandlingMode = sNodeMap.GetNode("StreamBufferHandlingMode");
        CHECK_ARW(ptrHandlingMode);
        Spinnaker::GenApi::CEnumEntryPtr ptrHandlingModeEntry = ptrHandlingMode->GetEntryByName("NewestOnly");
        ptrHandlingMode->SetIntValue(ptrHandlingModeEntry->GetValue());

        // Enable PTP
        // Spinnaker::GenApi::CBooleanPtr ptrPtpEnable = nodeMap.GetNode("PtpEnable");
        // CHECK_ARW(ptrPtpEnable);
        // ptrPtpEnable->SetValue(true);
        // std::cout << "PTP habilitado en la cámara." << std::endl;

        // Setup camera as slave in PTP
        // Spinnaker::GenApi::CEnumerationPtr ptrPtpRole = nodeMap.GetNode("PtpRole");
        // CHECK_ARW(ptrPtpRole);
        // Spinnaker::GenApi::CEnumEntryPtr ptrPtpRoleSlave = ptrPtpRole->GetEntryByName("Slave");
        // CHECK_AR(ptrPtpRoleSlave);
        // const int64_t ptpRoleSlave = ptrPtpRoleSlave->GetValue();
        // ptrPtpRole->SetIntValue(ptpRoleSlave);


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
        std::cout << "[FlirAdapter::beginAcquisition] End acquisition." << std::endl;
        pFlir->EndAcquisition();
    }
    else
    {
        std::cout << "[FlirAdapter::beginAcquisition] Acquisition is not running." << std::endl;
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
bool acquireImage(cv::Mat& image, uint64_t& timestamp)
{
    CHECK_POINTER(pFlir);
    bool result = true;

    Spinnaker::ImagePtr pResultImage = nullptr;
    try
    {
        pResultImage = pFlir->GetNextImage(10000);
        if (!pResultImage)
        {
            std::cout << "[FlirAdapter::acquireImage] No grab result reference." << std::endl;
            return false;
        }
        if (pResultImage->IsIncomplete())
        {
            std::cerr << "[FlirAdapter::acquireImage] Image incomplete with image status " << pResultImage->GetImageStatus() << std::endl;
            result =  false;
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
            image = cv::Mat(rows, cols, (num_channels == 3) ? CV_8UC3 : CV_8UC1, image_data, stride);
            timestamp = convertedImage->GetTimeStamp();
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
 * @brief Function that handle all camera de-initializacoin, port closing and Spinnaker clean finishing.
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