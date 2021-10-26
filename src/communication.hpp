#include <opencv2/opencv.hpp>
#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

namespace comm
{
    class cameraInterface
    {
        public:
            cameraInterface();
            void cameraCallback(ConstImageStampedPtr &msg);
            cv::Mat checkImage();
            bool receptionAccomplished();

        private:
            cv::Mat image;
            bool imageReceived;
    };

}