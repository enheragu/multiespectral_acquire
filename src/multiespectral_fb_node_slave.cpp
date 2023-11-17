#include "ros/ros.h"

#include <sstream>

#include "camera_adapter.h"
std::string IMAGE_PATH; 


int main(int argc, char **argv)
{
    ros::init(argc, argv, "MultiespectralSlaveAcquire");
    ros::NodeHandle n;

    /**
     * Prepare image folder
     */
    int frame_rate;
    ros::param::param<std::string>("dataset_output_path", IMAGE_PATH, "./");
    ros::param::param<int>("frame_rate", frame_rate, 1);

    bool result = initCamera(frame_rate);
    result = result | setAsSlave();
    ROS_FATAL_STREAM_COND(!result, "[MultiespectralSlave] Could not initialize " << getName() << " camera.");
    if (!result) ros::shutdown();

    cv::Mat curr_image;
    int img_index = 0;
    while (ros::ok())
    {
        result =  acquireImage(curr_image);
        if (result)
        {
            std::ostringstream filename;
            filename << IMAGE_PATH << getType() << std::to_string(img_index) << ".jpg";
            cv::imwrite(filename.str().c_str(), curr_image);

            img_index += 1;
        }
        ROS_ERROR_STREAM_COND(!result, "[MultiespectralSlave] Could not acquire image from " << getName() << " camera.");
            
        ros::spinOnce();
    }


    result = closeCamera();
    ROS_FATAL_STREAM_COND(!result, "[MultiespectralSlave] Could finish correctly " << getName() << " camera.");
    return result;
}
