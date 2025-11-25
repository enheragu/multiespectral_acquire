/**
 * This file is a wrapper of Basler API for GeniCam camera acA 1600-60gc, exposes a common API to be wrapped into a
 * ROS node.
 */
#include <memory>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

// Basler API
#include <pylon/PylonIncludes.h>
#include "a2A2600_20gcBAS_pylon_gen/BaslerCamera.h"
#include "a2A2600_20gcBAS_pylon_gen/BaslerCameraArray.h"

#include "camera_adapter.h"

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
std::unique_ptr<Pylon::BaslerCamera> pBasler;
std::string camera_name = "Default:acA1600-60gc";
/**
 * @brief Get name of the camera for logging purposes
 */
std::string getName()
{
    if(pBasler)
    {
        camera_name = std::string(pBasler->GetDeviceInfo().GetFriendlyName())+std::string(":")+std::string(pBasler->GetDeviceInfo().GetModelName());
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
        Pylon::PylonInitialize();
        // Basler with IP: '192.168.4.5'
        // This takes first abailable
        // pBasler = std::unique_ptr<Pylon::BaslerCamera>(new Pylon::BaslerCamera(Pylon::CTlFactory::GetInstance().CreateFirstDevice()));
        

        Pylon::CTlFactory& tl_factory = Pylon::CTlFactory::GetInstance();
        Pylon::DeviceInfoList_t device_list;
        
        if (0 == tl_factory.EnumerateDevices(device_list))
        {
            std::cerr << "[BaslerAdapter::initCamera] No available camera devices.";
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
                    std::cout << "[BaslerAdapter::initCamera] Found camera device:"
                    << " Device Model: " << it->GetModelName() << "; "
                    << " with Device User Id: " << it->GetUserDefinedName();
                    
                    pBasler = std::unique_ptr<Pylon::BaslerCamera>(new Pylon::BaslerCamera(tl_factory.CreateDevice(*it)));
                    found_desired_device = true;
                    break;
                }
            }

            if (!found_desired_device)
            {
                std::cerr << "[BaslerAdapter::initCamera] Could not found camera with id: " << camera_ip << std::endl;
                return false;
            }
        }

        // Not working :( creates camera that cannot get images
        // This access it by IP
        // Pylon::CTlFactory& tlFactory = Pylon::CTlFactory::GetInstance(); 
        // Pylon::CDeviceInfo info; 
        // info.SetDeviceClass(Pylon::BaslerGigEDeviceClass); 
        // info.SetIpAddress(camera_ip.c_str()); 
        // Pylon::IPylonDevice* device = tlFactory.CreateDevice(info);
        // pBasler = std::unique_ptr<Pylon::BaslerCamera>(new Pylon::BaslerCamera(device));

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
            std::cout << "\t\t· IP configured: " << pBasler->GetDeviceInfo().GetIpAddress() << std::endl;
            std::cout << "\t\t· Requested IP: " << camera_ip << std::endl;
        }

        pBasler->Open();
        
        // pBasler->Height.TrySetToMaximum();
        // pBasler->OffsetX.TrySetToMinimum();
        // pBasler->OffsetY.TrySetToMinimum();
        // pBasler->Width.TrySetToMaximum();
        
        // GenApi::CEnumerationPtr(pBasler->GetNodeMap().GetNode("AcquisitionMode"))->FromString("Continuous");
        // GenApi::CEnumerationPtr triggerMode = pBasler->GetNodeMap().GetNode("TriggerMode");

        // GenApi::CEnumerationPtr(pBasler->GetNodeMap().GetNode("AcquisitionMode"))->FromString("Continuous");


        // GenApi::CEnumerationPtr triggerMode = pBasler->GetNodeMap().GetNode("TriggerMode");
        // CHECK_AW(triggerMode);
        // triggerMode->FromString("Off");

        // GenApi::CEnumerationPtr triggerSource = pBasler->GetNodeMap().GetNode("TriggerSource");
        // CHECK_AW(triggerSource);
        // triggerSource->FromString("Software");

        // GenApi::CEnumerationPtr triggerSelector = pBasler->GetNodeMap().GetNode("TriggerSelector");
        // CHECK_AW(triggerSelector);
        // triggerSelector->FromString("FrameStart");


        // CHECK_ARW(pBasler->ExposureAuto);
        // pBasler->ExposureAuto.SetValue(Pylon::BaslerCameraCameraParams_Params::ExposureAuto_Continuous);
        // std::cout << "pBasler->ExposureAuto = " << pBasler->ExposureAuto.ToStringOrDefault("<not readable>") << std::endl;
        // CHECK_ARW(pBasler->ExposureMode);
        // pBasler->ExposureMode.SetValue(Pylon::BaslerCameraCameraParams_Params::ExposureMode_Timed);
        // std::cout << "pBasler->ExposureMode = " << pBasler->ExposureMode.ToStringOrDefault("<not readable>") << std::endl;

        // With autoexposure the camera is not triggering?¿
        // Enable Auto Exposure (set to Continuous mode) Need to reconfigure these other values to 
        // be able to set autoexposure
        // CHECK_ARW(pBasler->AutoExposureTimeLowerLimit);
        // pBasler->AutoExposureTimeLowerLimit.SetValue(100.0f);
        // pBasler->AutoExposureTimeUpperLimit.SetValue(100000.0f);

        // CHECK_ARW(pBasler->AutoTargetBrightness);
        // pBasler->AutoTargetBrightness.SetValue(0.8f);
       
        GenApi::CEnumerationPtr(pBasler->GetNodeMap().GetNode("TriggerMode"))->FromString("Off");
        pBasler->AcquisitionMode.TrySetValue( Pylon::BaslerCameraCameraParams_Params::AcquisitionMode_Continuous );
        
        CHECK_ARW(pBasler->ExposureAuto);
        pBasler->ExposureAuto.SetValue(Pylon::BaslerCameraCameraParams_Params::ExposureAuto_Continuous);
        std::cout << "[BaslerAdapter::initCamera] Autoexposure enabled in continuous mode." << std::endl;

        pBasler->AcquisitionFrameRateEnable.SetValue(true);
        pBasler->AcquisitionFrameRate.SetValue(frame_rate);

        // Enable PTP and set camera as slave
        // pBasler->GevIEEE1588.SetValue(true);

        std::cout << "pBasler->AcquisitionMode = " << pBasler->AcquisitionMode.ToStringOrDefault("<not readable>") << std::endl;
        std::cout << "pBasler->TriggerMode = " << pBasler->TriggerMode.ToStringOrDefault("<not readable>") << std::endl;
        std::cout << "pBasler->TriggerSelector = " << pBasler->TriggerSelector.ToStringOrDefault("<not readable>") << std::endl;

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
    pBasler->StartGrabbing(Pylon::GrabStrategy_LatestImageOnly);
    return true;
}

/**
 * @brief Function that handle acquisition end.
 */
bool endAcquisition()
{
    CHECK_POINTER(pBasler);
    pBasler->StopGrabbing();
    return true;
}

/**
 * @brief Configure camera as Master to be synchronized through hardware trigger
 * @return true or false depending on image acquisition
 */
bool setAsMaster()
{
    std::cout << "No master configuration for now :)" << std::endl;
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
bool acquireImage(cv::Mat& image, uint64_t& timestamp, ImageMetadata& metadata)
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

        // Wait for an image and then retrieve it. A timeout of 1000 ms is used.
        pBasler->RetrieveResult( 2000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);
        
        // Image grabbed successfully?
        if (!ptrGrabResult)
        {
            std::cout << "[BaslerAdapter::acquireImage] No grab result reference." << std::endl;
            return false;
        }
        if (ptrGrabResult->GrabSucceeded())
        {
            // Access the image data.
            const uint8_t *pImageBuffer = (uint8_t *) ptrGrabResult->GetBuffer();

            formatConverter.Convert(pylonImage, ptrGrabResult);
            // needs to be cloned so to not keep pointing to local raw data that will be destroyed after function finishes
            image = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t *)pylonImage.GetBuffer()).clone();
            timestamp = ptrGrabResult->GetTimeStamp();
        }
        else
        {
            std::cout << "[BaslerAdapter::acquireImage] Error: " << ptrGrabResult->GetErrorCode() << " " << ptrGrabResult->GetErrorDescription() << std::endl;
            return false;
        }
    }
    catch (const Pylon::GenericException &e)
    {
        // Error handling.
        std::cerr << "[BaslerAdapter::acquireImage] Pylon exception: " << e.GetDescription() << std::endl;
        return false;
    }
    return true;   
}


// bool acquireImage(cv::Mat& image, uint64_t& timestamp)
// {
//     try
//     {
//         // Ensure grabbing is stopped if it was previously started
//         if (pBasler->IsGrabbing())
//         {
//             pBasler->StopGrabbing();
//         }

//         // Start grabbing
//         pBasler->StartGrabbing(1); // Grab one frame

//         // Wait for grab result
//         Pylon::CGrabResultPtr ptrGrabResult;
//         // This smart pointer will receive the grab result data.
//         cv::Mat openCvImage;
//         Pylon::CImageFormatConverter formatConverter;
//         formatConverter.OutputPixelFormat = Pylon::PixelType_BGR8packed;
//         Pylon::CPylonImage pylonImage;

//         if (pBasler->RetrieveResult(5000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException))
//         {
//             // Process the image
//             if (ptrGrabResult->GrabSucceeded())
//             {
//                 // Access the image data.
//                 const uint8_t *pImageBuffer = (uint8_t *) ptrGrabResult->GetBuffer();

//                 formatConverter.Convert(pylonImage, ptrGrabResult);
//                 // needs to be cloned so to not keep pointing to local raw data that will be destroyed after function finishes
//                 image = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t *)pylonImage.GetBuffer()).clone();
//                 timestamp = ptrGrabResult->GetTimeStamp();
//                 return true;
//             }
//             else
//             {
//                 std::cerr << "[BaslerAdapter::acquireImage] Grab failed." << std::endl;
//                 return false;
//             }
//         }
//         else
//         {
//             std::cerr << "[BaslerAdapter::acquireImage] Grab timed out." << std::endl;
//             return false;
//         }
//     }
//     catch (const Pylon::GenericException &e)
//     {
//         std::cerr << "[BaslerAdapter::acquireImage] Pylon exception: " << e.GetDescription() << std::endl;
//         return false;
//     }
// }


/**
 * @brief Function that handle all camera de-initializacoin, port closing and Pylon clean finishing.
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