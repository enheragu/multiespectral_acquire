#ifndef MULTISPECTRAL_PLUGIN_H
#define MULTISPECTRAL_PLUGIN_H

#include <ros/ros.h>
#include <rviz/panel.h>
#include <actionlib/client/simple_action_client.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <QPushButton>
#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTimer>
#include <QComboBox>
#include <QTime>
#include <pluginlib/class_list_macros.h>

#include <multiespectral_fb/MultiespectralAcquisitionAction.h>

class MultiespectralPlugin : public rviz::Panel
{
Q_OBJECT
public:
    MultiespectralPlugin(QWidget* parent = 0);
Q_SIGNALS: 
    void imageReceived();
    
public Q_SLOTS:
    void sendGoal();
    void cancelGoal();
    void imageCallbackLWIR(const sensor_msgs::ImageConstPtr& msg);
    void imageCallbackRGB(const sensor_msgs::ImageConstPtr& msg);
    void doneCb(const actionlib::SimpleClientGoalState& state, const multiespectral_fb::MultiespectralAcquisitionResultConstPtr& result);
    void activeCb();
    void feedbackCb(const multiespectral_fb::MultiespectralAcquisitionFeedbackConstPtr& feedback);
    void updateImages();

private:
    void initializeImages();

    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber lwir_sub, rgb_sub;
    actionlib::SimpleActionClient<multiespectral_fb::MultiespectralAcquisitionAction> ac_lwir;
    actionlib::SimpleActionClient<multiespectral_fb::MultiespectralAcquisitionAction> ac_rgb;
    QPushButton* start_button;
    QPushButton* cancel_button;
    QLabel* lwir_label;
    QLabel* rgb_label;
    QLabel* frequency_label;
    QLabel* num_images_label;
    QVBoxLayout* layout;
    QHBoxLayout* controls_layout;
    QHBoxLayout* images_layout;
    QVBoxLayout* info_layout;
    QTimer* timer;
    QImage img_lwir;
    QImage img_rgb;
    std::string action_name;
    std::string lwir_topic;
    std::string rgb_topic;
    int lwir_frame_count;
    int rgb_frame_count;
    int lwir_total_frame_count = 0;
    int rgb_total_frame_count = 0;
    QTime frame_timer;
};

#endif // MULTISPECTRAL_PLUGIN_H
