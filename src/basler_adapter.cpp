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

// Reference to basler camera to be handled
std::unique_ptr<Pylon::BaslerCamera> pBasler;

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
 * @return true or false depending on image acquisition
 */
bool initCamera(int frame_rate)
{
    try
    {
        // Before using any pylon methods, the pylon runtime must be initialized. 
        Pylon::PylonInitialize();
        pBasler = std::unique_ptr<Pylon::BaslerCamera>(new Pylon::BaslerCamera(Pylon::CTlFactory::GetInstance().CreateFirstDevice()));

        pBasler->StartGrabbing();
        return true;
    }
    catch (const Pylon::GenericException &e)
    {
        std::cerr << "[BaslerAdapter::initCamera] Pylon exception: " << e.GetDescription() << std::endl;
        return false;
    }
}

/**
 * @brief Configure camera as Master to be synchronized through hardware trigger
 * @return true or false depending on image acquisition
 */
bool setAsMaster()
{
    return true;   
}

/**
 * @brief Configure camera as Slave to be synchronized through hardware trigger
 * @return true or false depending on image acquisition
 */
bool setAsSlave()
{
    return true;   
}

/**
 * @brief Function that handles image acquisition. Returns image in CV format.
 * @param image CV mat reference to be filled with image
 * @return true or false depending on image acquisition
 */
bool acquireImage(cv::Mat& image)
{
    if (!pBasler)
    {
            std::cout << "[BaslerAdapter::acquireImage] No camera pointer available." << std::endl;
            return false;
    }
    try
    {
        // This smart pointer will receive the grab result data.
        Pylon::CGrabResultPtr ptrGrabResult;
        cv::Mat openCvImage;

        Pylon::CImageFormatConverter formatConverter;
        formatConverter.OutputPixelFormat = Pylon::PixelType_BGR8packed;
        Pylon::CPylonImage pylonImage;

        // Wait for an image and then retrieve it. A timeout of 100 ms is used.
        pBasler->RetrieveResult( 100, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);
        
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
    pBasler->StopGrabbing();
    pBasler->Close();
    pBasler.reset();

    // Releases all pylon resources. 
    Pylon::PylonTerminate(); 

    return true;
}