/**
 * This file is a wrapper of Basler API for GeniCam camera acA 1600-60gc, exposes a common API to be wrapped into a
 * ROS node.
 */

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

// Basler API
#include <pylon/PylonIncludes.h>
#include "BaslerCamera.h"
#include "BaslerCameraArray.h"

// Reference to basler camera to be handled
Pylon::BaslerCamera& camera;

/**
 * @brief Get name of the camera for logging purposes
 */
std::string getName()
{
    return "Basler acA1600-60gc"
}

/**
 * @brief Function that handle all Pylon and Camera initializacion and configuration.
 * @return true or false depending on image acquisition
 */
bool initCamera(int frame_rate)
{
    // Before using any pylon methods, the pylon runtime must be initialized. 
    Pylon::PylonInitialize();
    camera = Pylon::BaslerCamera(Pylon::CTlFactory::GetInstance().CreateFirstDevice());
    return true;
}

/**
 * @brief Configure camera as Master to be synchronized through hardware trigger
 * @return true or false depending on image acquisition
 */
bool setAsMaster()
{

}

/**
 * @brief Configure camera as Slave to be synchronized through hardware trigger
 * @return true or false depending on image acquisition
 */
bool setAsSlave()
{

}

/**
 * @brief Function that handles image acquisition. Returns image in CV format.
 * @param image CV mat reference to be filled with image
 * @return true or false depending on image acquisition
 */
bool acquireImage(cv::Mat& image)
{
    pBasler.StartGrabbing(1);
    // This smart pointer will receive the grab result data.
    Pylon::CGrabResultPtr ptrGrabResult;
    cv::Mat openCvImage;

    Pylon::CImageFormatConverter formatConverter;
    formatConverter.OutputPixelFormat = Pylon::PixelType_BGR8packed;
    Pylon::CPylonImage pylonImage;

    // Wait for an image and then retrieve it. A timeout of 5000 ms is used.
    pBasler.RetrieveResult( 5000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);
    
    // Image grabbed successfully?
    if (ptrGrabResult->GrabSucceeded())
    {
        // Access the image data.
        std::cout << "SizeX: " << ptrGrabResult->GetWidth() << std::endl;
        std::cout << "SizeY: " << ptrGrabResult->GetHeight() << std::endl;
        const uint8_t *pImageBuffer = (uint8_t *) ptrGrabResult->GetBuffer();
        std::cout << "Gray value of first pixel: " << (uint32_t) pImageBuffer[0]  << std::endl;

        formatConverter.Convert(pylonImage, ptrGrabResult);
        image = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t *)pylonImage.GetBuffer());
    }
    else
    {
        std::cout << "Error: " << ptrGrabResult->GetErrorCode() << " " << ptrGrabResult->GetErrorDescription() << std::endl;
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
    // Deinitialize Basler
    camera.Close();

    // Releases all pylon resources. 
    Pylon::PylonTerminate(); 

    return true;
}