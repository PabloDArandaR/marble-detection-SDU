#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

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