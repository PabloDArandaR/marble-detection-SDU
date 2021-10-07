#include <opencv2/opencv.hpp>

namespace comm
{

    class cameraInterface
    {
        public:
            void cameraCallback(ConstImageStampedPtr &msg);
            cv::Mat checkImage();

        private:
            cv::Mat image;
    }

}