#include <opencv2/opencv.hpp>
#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <algorithm>

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
}

#endif