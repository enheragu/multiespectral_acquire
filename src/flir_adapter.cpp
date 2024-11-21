/**
 * This file is a wrapper of Flir API for GeniCam camera acA 1600-60gc, exposes a common API to be wrapped into a
 * ROS node.
 */

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

// Reference to Flir camera to be handled
Spinnaker::CameraPtr pFlir = nullptr;
Spinnaker::CameraList flirCamList;
Spinnaker::SystemPtr flir_system;

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
 * @brief Function that handle all Basler initializacion and configuration.
 * @return true or false depending on image acquisition
 */
bool initCamera(int frame_rate)
{
    bool result = false;
    try
    {
        flir_system = Spinnaker::System::GetInstance();

        flirCamList = flir_system->GetCameras();
        if (flirCamList.GetSize()<=0)
        {
            std::cerr << "[FlirAdapter::initCamera] No cameras detected."  << std::endl;
            return false;
        }

        std::cerr << "[FlirAdapter::initCamera] "<<flirCamList.GetSize()<<" cameras detected."  << std::endl;
        // pFlir = flirCamList.GetByIndex(0);
        pFlir = flirCamList.GetBySerial("M0000726");
        pFlir->Init();

        // Continuous Acquisition
        CHECK_ARW(pFlir->AcquisitionMode);
        pFlir->AcquisitionMode.SetValue(Spinnaker::AcquisitionMode_Continuous);
        
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
    pFlir->BeginAcquisition();
    return true;
}

/**
 * @brief Configure camera as Master to be synchronized through hardware trigger
 * @return true or false depending on image acquisition
 */
bool setAsMaster()
{
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
bool acquireImage(cv::Mat& image)
{
    if (!pFlir)
    {
            std::cout << "[FlirAdapter::acquireImage] No camera pointer available." << std::endl;
            return false;
    }

    bool result = true;
    try
    {
        Spinnaker::ImagePtr pResultImage = pFlir->GetNextImage(1000);
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
            pResultImage = processor.Convert(pResultImage, Spinnaker::PixelFormat_Mono8);

            unsigned int rows = pResultImage->GetHeight();
            unsigned int cols = pResultImage->GetWidth();
            unsigned int num_channels = pResultImage->GetNumChannels();
            void *image_data = pResultImage->GetData();
            unsigned int stride = pResultImage->GetStride();
            image = cv::Mat(rows, cols, (num_channels == 3) ? CV_8UC3 : CV_8UC1, image_data, stride);
        }      
        pResultImage->Release();  
    }
    catch (Spinnaker::Exception& e)
    {
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
        if (pFlir->IsStreaming()) pFlir->EndAcquisition(); 
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