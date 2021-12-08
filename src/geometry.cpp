#include <opencv2/opencv.hpp>
#include <iostream>
#include "vectorMod.cpp"
#include "math.h"

#ifndef GEOMETRY_DEFINE_CPP
#define GEOMETRY_DEFINE_CPP

std::vector<std::string> split(const std::string &text, char sep) {
    std::vector<std::string> tokens;
    std::size_t start = 0, end = 0;
    while ((end = text.find(sep, start)) != std::string::npos) {
        if (end != start) {
          tokens.push_back(text.substr(start, end - start));
        }
        start = end + 1;
    }
    if (end != start) {
       tokens.push_back(text.substr(start));
    }
    return tokens;
}

double distanceToMarble(circleAvg input, float realRadius, float f)
{
    return {(f/input.radius)*realRadius};
}

double veryCustomDistance(double x1, double y1, double z1, double x2, double y2, double z2)
{
    double x = (x1-x2)*(x1-x2);
    double y = (y1-y2)*(y1-y2);
    double z = (z1-z2)*(z1-z2);

    return sqrt(x + y + z);
}

double angleToPointRad(float input, int width, float f)
{
    return {atan((width/2 - input)/f)};
}


double angleToPointDeg(float input, int width, float f)
{
    double pi = 3.14159265;
    return {angleToPointRad(input, width, f)*180/pi};
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

#endif