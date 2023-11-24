/**
 * Header for basler_adapter.cpp and flir_adapter.cpp
 */
#ifndef CAMERA_ADAPTER_H
#define CAMERA_ADAPTER_H


#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

std::string getName();
std::string getType();
bool initCamera(int frame_rate);
bool beginAcquisition();
bool setAsMaster();
bool setAsSlave();
bool acquireImage(cv::Mat& image);
bool closeCamera();


class MultiespectralAcquireT
{
protected:
    int img_stored = 0;
    std::string img_path = "";
    std::mutex camera_mutex; // Avoid deinitialization while grabbing image
public:
    MultiespectralAcquireT(std::string img_path) : img_path(img_path) {}
    ~MultiespectralAcquireT(void);
    bool init(int frame_rate);
    bool grabStoreImage();
};

#endif //CAMERA_ADAPTER_H