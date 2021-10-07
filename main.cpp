#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <opencv2/opencv.hpp>

#include <iostream>

#include "src/filter.hpp"

int main()
{
    const double stdDev {0.02};
    const double mean {0};
    const int kernel_size {3};
    cv::Mat image, image_gray, filtered;

    while(true)
    {
        // Read the image from the node

        // Initial filtering of the image
        cv::cvtColor(image, image_gray, cv::COLOR_RGB2GRAY);
        filtered = filter::gaussFilter(image_gray, mean, 0.02, kernel_size);

        // Segmentation/recognition

        // Control of the robot based on the images

    }

    return 0;
}