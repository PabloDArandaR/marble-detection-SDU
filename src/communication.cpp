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

        float px_per_m = 200 / this->elementReceived.range_max;
        int width = 400;
        int height = 400;

        cv::Mat im(height, width, CV_8UC3);
        im.setTo(0);

        for (int i = 0; i < this->elementReceived.nranges; i++)
        {
            float angle = this->elementReceived.angle_min + i * this->elementReceived.angle_increment;
            float range = std::min(float(msg->scan().ranges(i)), this->elementReceived.range_max);
            cv::Point2f startpt(200.5f + this->elementReceived.range_min * px_per_m * std::cos(angle),
                        200.5f - this->elementReceived.range_min * px_per_m * std::sin(angle));
            cv::Point2f endpt(200.5f + range * px_per_m * std::cos(angle),
                            200.5f - range * px_per_m * std::sin(angle));

            cv::line(im, startpt * 16, endpt * 16, cv::Scalar(255, 255, 255, 255), 1,
                    cv::LINE_AA, 4);
            

            if (this->elementReceived.ranges.size() <= i)
            {
                this->elementReceived.ranges.push_back({range, angle});
            }
            else
            {
                this->elementReceived.ranges[i] = {range, angle};
            }
        }
        cv::circle(im, cv::Point(200, 200), 2, cv::Scalar(0, 0, 255));
        this->elementReceived.im = im.clone();
        this->received = true;
    }

    void printMessage(ConstPosesStampedPtr &msg){
        for (int i = 0; i < msg->pose_size(); i++)
        {
            std::cout << "Name: " << msg->pose(i).name() << std::endl;
        }
    };

    void poseInterface::callbackMsg(ConstPosesStampedPtr &msg)
    {
        for (int i = 0; i < msg->pose_size(); i++)
        {
            // The center of the robot
            if (msg->pose(i).name() == "pioneer2dx")
            {
                this->elementReceived.x = msg->pose(i).position().x();
                this->elementReceived.y = msg->pose(i).position().y();
                this->elementReceived.z = msg->pose(i).position().z();
            }

            // The front of the robot
            if (msg->pose(i).name() == "pioneer2dx::camera::link"){
                this->front.x = msg->pose(i).position().x();
                this->front.y = msg->pose(i).position().y();
                this->front.z = msg->pose(i).position().z();
            }
        }

        this->front.x += this->elementReceived.x;
        this->front.y += this->elementReceived.y;
        this->front.z += this->elementReceived.z;

        received = true;
    }

    void marbleInterface::callbackMsg(ConstPosesStampedPtr &msg)
    {
        for (int i = 0; i < msg->pose_size(); i++)
        {
            if (msg->pose(i).name() == "marble::marble::link::link")
            {
                this->elementReceived.x = msg->pose(i).position().x();
                this->elementReceived.y = msg->pose(i).position().y();
                this->elementReceived.z = msg->pose(i).position().z();
            }
        }

        received = true;
    }

}

#endif