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
#include "BaslerCamera.h"
#include "BaslerCameraArray.h"

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

Pylon::String_t BASLER_IP = "169.254.165.5";

/**
 * @brief Get name of the camera for logging purposes
 */
std::string getName()
{
    return "Basler acA1600-60gc";
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
bool initCamera(int frame_rate)
{
    try
    {
        // Before using any pylon methods, the pylon runtime must be initialized. 
        Pylon::PylonInitialize();
        //Basler with IP: '192.168.4.5'
        // This takes first abailable
        // pBasler = std::unique_ptr<Pylon::BaslerCamera>(new Pylon::BaslerCamera(Pylon::CTlFactory::GetInstance().CreateFirstDevice()));
        
        
        // This access it by IP
        Pylon::CTlFactory& tlFactory = Pylon::CTlFactory::GetInstance(); 
        Pylon::CDeviceInfo info; 
        info.SetDeviceClass(Pylon::BaslerGigEDeviceClass); 
        info.SetIpAddress(BASLER_IP); 
        Pylon::IPylonDevice* device = tlFactory.CreateDevice(info);
        pBasler = std::unique_ptr<Pylon::BaslerCamera>(new Pylon::BaslerCamera(device));

        if (!pBasler)
        {
            std::cerr << "[BaslerAdapter::initCamera] Camera with configured IP ("<<BASLER_IP<<") was not found."  << std::endl;
            return false;
        }
        else
        {
            std::cerr << "[BaslerAdapter::initCamera] Opening camera with IP ("<<BASLER_IP<<")."  << std::endl;
        }

        pBasler->Open();
        
        // Enable Auto Exposure (set to Continuous mode)
        CHECK_ARW(pBasler->ExposureAuto)
        pBasler->ExposureAuto.SetIntValue(Pylon::BaslerCameraCameraParams_Params::ExposureAuto_Continuous);
        std::cout << "[BaslerAdapter::initCamera] Autoexposure enabled in continuous mode." << std::endl;

        pBasler->AcquisitionFrameRateEnable.SetValue(true);
        pBasler->AcquisitionFrameRateAbs.SetValue(frame_rate);

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
    pBasler->StartGrabbing();
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
    CHECK_POINTER(pBasler);
    try
    {
        // Select Line 2 (output line)
        CHECK_AW(pBasler->LineSelector);
        pBasler->LineSelector.SetValue(Pylon::BaslerCameraCameraParams_Params::LineSelector_Out1);

        // Set it as output
        CHECK_AW(pBasler->LineMode);
        pBasler->LineMode.SetValue(Pylon::BaslerCameraCameraParams_Params::LineMode_Output);

        // Set the source signal to User Output 1
        CHECK_AW(pBasler->LineSource);
        pBasler->LineSource.SetValue(Pylon::BaslerCameraCameraParams_Params::LineSource_ExposureActive);
    }
    catch (const Pylon::GenericException &e)
    {
        // Error handling.
        std::cerr << "[BaslerAdapter::setAsMaster] Pylon exception: " << e.GetDescription() << std::endl;
        return false;
    }
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
bool acquireImage(cv::Mat& image)
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
        pBasler->RetrieveResult( 1000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);
        
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