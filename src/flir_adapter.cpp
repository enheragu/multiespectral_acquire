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

    bool result = true;
    flir_system = Spinnaker::System::GetInstance();

    Spinnaker::CameraList flirCamList = flir_system->GetCameras();
    result = flirCamList.GetSize()>0?result:false;

    pFlir = flirCamList.GetByIndex(0);
    pFlir->Init();

    // Single acquisition
    Spinnaker::GenApi::INodeMap& nodeMap = pFlir->GetNodeMap();
    Spinnaker::GenApi::CEnumerationPtr ptrAcquisitionMode = nodeMap.GetNode("AcquisitionMode");
    Spinnaker::GenApi::CEnumEntryPtr ptrAcquisitionModeSingle = ptrAcquisitionMode->GetEntryByName("SingleFrame");
    const int64_t acquisitionModeSingle = ptrAcquisitionModeSingle->GetValue();
    ptrAcquisitionMode->SetIntValue(acquisitionModeSingle);

    pFlir->BeginAcquisition();
    flirCamList.Clear();

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
    try
    {
        Spinnaker::ImagePtr pResultImage = pFlir->GetNextImage(1000);
        unsigned int rows = pResultImage->GetHeight();
        unsigned int cols = pResultImage->GetWidth();
        unsigned int num_channels = pResultImage->GetNumChannels();
        void *image_data = pResultImage->GetData();
        unsigned int stride = pResultImage->GetStride();
        image = cv::Mat(rows, cols, (num_channels == 3) ? CV_8UC3 : CV_8UC1, image_data, stride);
        pResultImage->Release();
    }
    catch (Spinnaker::Exception& e)
    {
        // Error handling.
        std::cerr << "[FlirAdapter::acquireImage] Pylon exception: " << e.what() << std::endl;
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
    if (pFlir) { pFlir->DeInit();    pFlir = nullptr; }
    flir_system->ReleaseInstance();
    return true;
}