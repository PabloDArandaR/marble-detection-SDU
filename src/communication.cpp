#include <opencv2/opencv.hpp>
#include <algorithm>
#include <iostream>
#include "communication.hpp"

#ifndef COMM_CPP_GUARD
#define COMM_CPP_GUARD

namespace comm
{

    void cameraInterface::callbackMsg(ConstImageStampedPtr &msg)
    {
        std::size_t width = msg->image().width();
        std::size_t height = msg->image().height();
        const char *data = msg->image().data().c_str();
        cv::Mat im(int(height), int(width), CV_8UC3, const_cast<char *>(data));
        im = im.clone();
        cv::cvtColor(im, im, cv::COLOR_RGB2BGR);

        this->elementReceived = im;
        received = true;
    }

    void lidarInterface::callbackMsg(ConstLaserScanStampedPtr &msg) {

        this->elementReceived.angle_min = float(msg->scan().angle_min());
        this->elementReceived.angle_increment = float(msg->scan().angle_step());
        this->elementReceived.range_min = float(msg->scan().range_min());
        this->elementReceived.range_max = float(msg->scan().range_max());
        this->elementReceived.nranges = msg->scan().ranges_size();
        this->elementReceived.nintensities = msg->scan().intensities_size();

        for (int i = 0; i < this->elementReceived.nranges; i++)
        {
            if (this->elementReceived.ranges.size() <= i)
            {
                this->elementReceived.ranges.push_back(std::min(float(msg->scan().ranges(i)), this->elementReceived.range_max));
            }
            else
            {
                this->elementReceived.ranges[i] = std::min(float(msg->scan().ranges(i)), this->elementReceived.range_max);
            }
        }
        this->received = true;
    }
}

#endif