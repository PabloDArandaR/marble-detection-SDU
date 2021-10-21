#include <opencv2/opencv.hpp>

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