/**
 * This is a Dummy adapter that includes the definition of necessary functions to compile and run while doing nothing :)
 */


#include "camera_adapter.h"
#include "core/utils/image_metadata.h"

// Global variable to control PTP behavior (set by camera_handler_node)
bool force_disable_ptp = false;

bool aquisition_status = false;
int frame_id = 0;
std::string camera_name = "Default:Dummy";  // Display name (usually ROS node name)
std::string model_name = "DummyCamera";  // Camera model name

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
    return camera_name;
}

/**
 * @brief Get the camera model name
 */
std::string getModelName()
{
    return model_name;
}

/**
 * @brief Return image type to store it correctly
 */
std::string getType()
{
    return "dummy";
}



/**
 * @brief Function that handle all Basler initializacion and configuration.
 * @return true or false depending on image acquisition
 */
bool initCamera(int frame_rate, std::string camera_ip)
{   
    int result = false;

    std::cerr << "[DummyAdapter::initCamera] Dummy camera detected with " <<std::string(camera_ip) << " IP."  << std::endl;
    std::cout << "[DummyAdapter::initCamera] Acquisition mode set to continuous with " << std::to_string(frame_rate) << " frame rate." << std::endl;

    result = true;

    return result;
}

/**
 * @brief Function that handle acquisition init. Note that it has to start after all configuration is set.
 * @return true or false depending on image acquisition result
 */
bool beginAcquisition()
{
    if (!aquisition_status)
    {
        std::cout << "[DummyAdapter::beginAcquisition] Starting acquisition." << std::endl;
        aquisition_status = true;
    }
    else
    {
        std::cout << "[DummyAdapter::beginAcquisition] Acquisition already started." << std::endl;
    }
    return true;
}

/**
 * @brief Function that handle acquisition end.
 */
bool endAcquisition()
{
    if (aquisition_status)
    {
        std::cout << "[DummyAdapter::endAcquisition] End acquisition." << std::endl;
        aquisition_status = false;
    }
    else
    {
        std::cout << "[DummyAdapter::endAcquisition] Acquisition is not running." << std::endl;
    }
    return true;
}

/**
 * @brief Configure camera as Master to be synchronized through hardware trigger
 * @return true or false depending on image acquisition
 */
bool setAsMaster()
{
    std::cout << "[DummyAdapter::setAsMaster] Set as master called for Dummy camera" << std::endl;
    return true;
}   


/**
 * @brief Configure camera as Slave to be synchronized through hardware trigger
 * @return true or false depending on image acquisition
 */
bool setAsSlave() 
{
    std::cout << "[DummyAdapter::setAsSlave] Set as slave called for Dummy camera" << std::endl;
    return true;
}

/**
 * @brief Function that handles image acquisition. Returns image in CV format.
 * @param image CV mat reference to be filled with image
 * @return true or false depending on image acquisition
 */
bool acquireImage(cv::Mat& image, ImageMetadata& metadata)
{
    std::cout << "[DummyAdapter::acquireImage]" << std::endl;
    

    /****************************
    **   Set dummy metadata    **
    *****************************/
    // Set timestamps
    auto now = std::chrono::system_clock::now();
    auto nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
    metadata.camera_timestamp = static_cast<uint64_t>(nanos);
    
    metadata.frameCounter = frame_id++;
    metadata.width = 640;
    metadata.height = 480;
    metadata.pixelFormat = "CV_8UC1";
    metadata.exposureTime = 10000.0; // Exposure

    image = cv::Mat::zeros(metadata.height, metadata.width, CV_8UC1);

    return true;
}

/**
 * @brief Function that handle all camera de-initialization
 * @return true or false depending on image acquisition
 */
bool closeCamera()
{
    std::cout << "[DummyAdapter::closeCamera] Close camera requested." << std::endl;
    return true;
}