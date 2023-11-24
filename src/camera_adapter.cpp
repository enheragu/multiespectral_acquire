

#include <mutex>

#include "ros/ros.h"

#include "camera_adapter.h"

bool MultiespectralAcquireT::init(int frame_rate)
{
    bool result = initCamera(frame_rate);
    ROS_FATAL_STREAM_COND(!result, "[MultiespectralAcquireT] Could not initialize " << getName() << " camera.");
    return result;
}

MultiespectralAcquireT::~MultiespectralAcquireT(void)
{   
    const std::scoped_lock<std::mutex> lock(camera_mutex);
    bool result = closeCamera();
    ROS_FATAL_STREAM_COND(!result, "[MultiespectralAcquireT] Could not finish correctly " << getName() << " camera.");
    ROS_INFO_STREAM_COND(result, "[MultiespectralAcquireT] Correctly finished " << getName() << " camera.");
}

bool MultiespectralAcquireT::grabStoreImage()
{

    const std::scoped_lock<std::mutex> lock(camera_mutex);
    cv::Mat curr_image;
    bool result =  acquireImage(curr_image);
    if (result && !curr_image.empty()) 
    {
        std::ostringstream filename;
        filename << img_path << "/" << getType() << "/" << std::to_string(img_stored) << ".jpg";
        cv::imwrite(filename.str().c_str(), curr_image);
        img_stored +=1;
    }
    
    ROS_ERROR_STREAM_COND(!result, "[MultiespectralAcquireT::grabStoreImage] Could not acquire image from " << getName() << " camera.");
    return result;
}
