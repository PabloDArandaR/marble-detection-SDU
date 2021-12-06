#include <opencv2/opencv.hpp>
#include <iostream>
#include "vectorMod.cpp"

double distanceToMarble(circleAvg input, float realRadius, float f)
{
    return {(f/input.radius)*realRadius};
}

std::vector<double> calcCoord(cv::Matx33f K, double depth, int dx, int dy)
{
    std::vector<double> calc;

    // Intrinsic camera matrix for the raw (distorted) images.
    //     [fx  0 cx]
    // K = [ 0 fy cy]
    //     [ 0  0  1]

    double x = dx;
    double y = dy;

    double m_fx = K(0,0);
    double m_fy = K(1,1);
    double m_cx = K(0,2);
    double m_cy = K(1,2);
    double inv_fx = 1. / m_fx;
    double inv_fy = 1. / m_fy;

    double calc_z = depth; 
    double calc_x = (x - m_cx) * calc_z * inv_fx;
    double calc_y = (y - m_cy) * calc_z * inv_fy;

    calc.push_back(calc_x);
    calc.push_back(calc_y);
    calc.push_back(calc_z);

    return calc;
}