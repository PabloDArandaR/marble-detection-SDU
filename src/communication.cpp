#include <opencv2/opencv.hpp>

#include "communication.hpp"

namespace comm
{
    cameraInterface::cameraInterface() : imageReceived {false} 
        {};

    void cameraInterface::cameraCallback(ConstImageStampedPtr &msg)
    {
        std::size_t width = msg->image().width();
        std::size_t height = msg->image().height();
        const char *data = msg->image().data().c_str();
        cv::Mat im(int(height), int(width), CV_8UC3, const_cast<char *>(data));

        std::cout << "[CAMERA] Received image \n";
        std::cout << "[CAMERA] Dimensions of the image are:  " << int(height) << ", " << int(width) << std::endl;

        im = im.clone();
        cv::cvtColor(im, im, cv::COLOR_RGB2BGR);

        this->image = im;
    }

    cv::Mat cameraInterface::checkImage()
    {
        return this->image;
    }

    bool cameraInterface::receptionAccomplished()
    {
        return this->imageReceived;
    }
}