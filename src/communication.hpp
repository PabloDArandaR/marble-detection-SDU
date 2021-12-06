#include <opencv2/opencv.hpp>
#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <algorithm>
#include <math.h>

#ifndef COMM_GUARD
#define COMM_GUARD

namespace comm
{
    struct lidarMsg
    {
        float angle_min, angle_increment, range_min, range_max;
        int nranges, nintensities;
        std::vector<int> ranges;
    };

    struct point
    {
        float x,y,z;

        float distance(point input)
        {
            return sqrt(pow(input.x - x, 2) + pow(input.y - y, 2) + pow(input.z - z, 2));
        }
    };

    template <typename T>
    class Interface
    {
        public:
            Interface() : received{false} {};

            void callbackMsg();

            bool receptionAccomplished()
            {
                return this -> received;
            }
            
            T checkReceived()
            {
                return this -> elementReceived;
            }
        
        protected:
            bool received;
            T elementReceived;

    };

    class cameraInterface : public Interface<cv::Mat>
    {
        public:
            void callbackMsg(ConstImageStampedPtr &msg);
    };

    class lidarInterface : public Interface<lidarMsg>
    {
        public:
            void callbackMsg(ConstLaserScanStampedPtr &msg);
    };

    class poseInterface : public Interface<point>
    {
        public:
            void callbackMsg(ConstPosesStampedPtr &msg);
    };

    class marbleInterface : public Interface<point>
    {
        public:
            void callbackMsg(ConstPosesStampedPtr &msg);
    };
}

#endif