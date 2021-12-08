#include <opencv2/opencv.hpp>
#include <iostream>
#include <math.h>

#include "communication.cpp"
#include "geometry.cpp"


float lidarInfo(comm::lidarMsg & lidar, int width, float f, circleAvg marble){
    float distance = lidar.range_max;

    float px_per_m = 200 / lidar.range_max;
    double trans = 180/3.141592;

    double angle1 = angleToPointRad(marble.x_center - marble.radius, width, f);
    double angle2 = angleToPointRad(marble.x_center + marble.radius, width, f);

    for (auto range : lidar.ranges){

        if ((angle2 < range.angle) && (angle1 > range.angle)) {

            if (distance > range.range) {
                distance = range.range;
            }
            
            cv::Point2f startpt(200.5f + lidar.range_min * px_per_m * std::cos(range.angle),
                                200.5f - lidar.range_min * px_per_m * std::sin(range.angle));
            cv::Point2f endpt(200.5f + range.range * px_per_m * std::cos(range.angle),
                            200.5f - range.range * px_per_m * std::sin(range.angle));

            cv::line(lidar.im, startpt * 16, endpt * 16, cv::Scalar(0, 255, 255, 255), 1,
                    cv::LINE_AA, 4);
        } else if (angle1 < range.angle) {
            
            break;
        }
    }

    

    return distance;
}