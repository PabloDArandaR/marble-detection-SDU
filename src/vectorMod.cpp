#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

#ifndef VECTOR_GUARD_CPP
#define VECTOR_GUARD_CPP

struct circleAvg
{
    int x_center,y_center,nCircles,radius;

    circleAvg() : x_center (0), y_center (0), nCircles (0), radius (0) {};
};

struct circle
{
    int x_center, y_center, radius;

    circle() : x_center(0), y_center(0), radius(0) {};
};

circle lowPassFilter(std::vector<circleAvg> inputs, float cutoff, float sampleFrequency)
{
    circle result;

    return result;
}

circleAvg complementaryFilter(circleAvg inputs, circleAvg past_circle, float beta)
{
    circleAvg result;

    result.x_center = beta * past_circle.x_center + (1 - beta) * inputs.x_center;
    result.y_center = beta * past_circle.y_center + (1 - beta) * inputs.y_center;
    result.radius = beta * past_circle.radius + (1 - beta) * inputs.radius;
    
    return result;
}

template <typename T>
void pushBeginning(std::vector<T> & inputVector, T inputValue, int maxLength)
{
    if(inputVector.size() < maxLength)
    {
        inputVector.push_back(inputValue);
    }
    else
    {
        for(int i = 0; i < maxLength - 1; i++)
        {
            inputVector[i] = inputVector[i-1];
        }
        inputVector[maxLength - 1] = inputValue;
    }
}

circleAvg averageCircle(std::vector<cv::Vec3f> input)
{
    circleAvg result;
    if (input.size() > 0)
    {
        result.nCircles = input.size();
        for(int i = 0; i < result.nCircles; i++)
        {
            result.x_center += input[i][0];
            result.y_center += input[i][1];
            result.radius += input[i][2];
        }

        result.x_center /= result.nCircles;
        result.y_center /= result.nCircles;
        result.radius /= result.nCircles;
    }

    return result;
}

#endif