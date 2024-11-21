#include <ros/ros.h>
#include <rviz/panel.h>
#include <actionlib/client/simple_action_client.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <QPushButton>
#include <pluginlib/class_list_macros.h>

#include <multiespectral_fb/MultiespectralAcquisitionAction.h>

class MultiespectralPlugin : public rviz::Panel
{
Q_OBJECT
public:
    MultiespectralPlugin(QWidget* parent = 0) : rviz::Panel(parent), it(nh), ac("my_action", true)
    {
        QPushButton* button = new QPushButton("Start Action", this);
        connect(button, SIGNAL(clicked()), this, SLOT(sendGoal()));

        cam1_sub = it.subscribe("/lwir_image", 1, &MultiespectralPlugin::imageCallback, this);
        cam2_sub = it.subscribe("/visible_image", 1, &MultiespectralPlugin::imageCallback, this);
    }

private Q_SLOTS:
    void sendGoal()
    {
        multiespectral_fb::MultiespectralAcquisitionGoal goal;
        ac.sendGoal(goal,
            boost::bind(&MultiespectralPlugin::doneCb, this, _1, _2),
            boost::bind(&MultiespectralPlugin::activeCb, this),
            boost::bind(&MultiespectralPlugin::feedbackCb, this, _1));
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        try
        {
            cv::imshow("Camera View", cv_bridge::toCvShare(msg, "bgr8")->image);
            cv::waitKey(30);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
    }

    void doneCb(const actionlib::SimpleClientGoalState& state, const multiespectral_fb::MultiespectralAcquisitionResultConstPtr& result)
    {
        ROS_INFO("Finished in state [%s]", state.toString().c_str());
        ROS_INFO("Result: %d", result->images_acquired);
    }

    void activeCb()
    {
        ROS_INFO("Goal just went active");
    }

    void feedbackCb(const multiespectral_fb::MultiespectralAcquisitionFeedbackConstPtr& feedback)
    {
        ROS_INFO("Got Feedback: %d", feedback->images_acquired);
    }

    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber cam1_sub, cam2_sub;
    actionlib::SimpleActionClient<multiespectral_fb::MultiespectralAcquisitionAction> ac;
};


PLUGINLIB_EXPORT_CLASS(MultiespectralPlugin, rviz::Panel)
