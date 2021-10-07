#include <opencv2/opencv.hpp>
#include "filter.hpp"

namespace filter
{
    cv::Mat gaussFilter(cv::Mat input, double mean, double stdDev, int kernel_size)
    {
        cv::Mat filtered;

        cv::GaussianBlur(input, filtered, cv::Size(kernel_size, kernel_size), stdDev);

        return filtered;
    }
}