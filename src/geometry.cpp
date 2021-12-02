#include <opencv2/opencv.hpp>
#include <iostream>
#include "vectorMod.cpp"

double distanceToMarble(circleAvg input, float realRadius, float f)
{
    return {(f/input.radius)*realRadius};
}