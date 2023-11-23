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
        pFlir = flirCamList.GetByIndex(0);
        pFlir->Init();

        // Single acquisition
        Spinnaker::GenApi::INodeMap& nodeMap = pFlir->GetNodeMap();
        Spinnaker::GenApi::CEnumerationPtr ptrAcquisitionMode = nodeMap.GetNode("AcquisitionMode");
        if (!IsReadable(ptrAcquisitionMode) ||
            !IsWritable(ptrAcquisitionMode))
        {
            std::cerr << "[FlirAdapter::initCamera] Unable to set acquisition mode. Node is not readable or writable. Aborting..." << std::endl;
            return false;
        }
        
        // TBD set aquisition mode correcly
        Spinnaker::GenApi::CEnumEntryPtr ptrAcquisitionModeSingle = ptrAcquisitionMode->GetEntryByName("Continuous"); //SingleFrame");
        const int64_t acquisitionModeSingle = ptrAcquisitionModeSingle->GetValue();
        ptrAcquisitionMode->SetIntValue(acquisitionModeSingle);

        pFlir->BeginAcquisition();
        result = true;
    }
    catch (Spinnaker::Exception& e)
    {
        // Error handling.
        std::cerr << "[FlirAdapter::initCamera] Pylon exception: " << e.what() << std::endl;
        return false;
    }
    return result;
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
    if (!pFlir)
    {
            std::cout << "[FlirAdapter::acquireImage] No camera pointer available." << std::endl;
            return false;
    }

    bool result = true;
    try
    {
        Spinnaker::ImagePtr pResultImage = pFlir->GetNextImage(100);
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
        std::cerr << "[FlirAdapter::acquireImage] Spinnaker exception: " << e.what() << std::endl;
        return false;
    }
    return result;   
}

/**
 * @brief Function that handle all camera de-initializacoin, port closing and Pylon clean finishing.
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