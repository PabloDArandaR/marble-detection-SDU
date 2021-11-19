#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <stdlib.h>  // for strtol

#include "src/communication.cpp"
#include "src/vectorMod.cpp"

static boost::mutex mutex;
const int key_left = 81;
const int key_up = 82;
const int key_down = 84;
const int key_right = 83;
const int key_esc = 27;

const int maxParam1 {300};
const int minParam1 {1};
const double incrementParam {1};
int param1 {100};
int param2 {35};
const int maxRadius {200};
const int minRadius {20};
const int maxLengthCircleVector {20};

int main(int argc, char ** argv)
{
    // Declaration of global variables
    double stdDev {0.02};
    const double mean {0};
    const int kernel_size {7};
    bool show_image {true};
    std::vector<std::vector<cv::Vec3f>> lastCircles;
    std::vector<circleAvg> meanCenter;
    cv::Mat image, image_gray, filtered;
    comm::cameraInterface * camera (new comm::cameraInterface);
    comm::lidarInterface * lidar (new comm::lidarInterface);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Argc check
    if (argc > 1)
    {
        show_image = std::strtol(argv[1], nullptr,10);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Setup Gazebo and ROS Subscriptions and Publishers

    // Load Gazebo's client
    gazebo::client::setup(argc, argv);
    // Creation of node for communication
    gazebo::transport::NodePtr node (new gazebo::transport::Node());
    node->Init();

    // Create subscribers to listen to Gazebo topics
    gazebo::transport::SubscriberPtr cameraSubscriber = node->Subscribe("~/pioneer2dx/camera/link/camera/image", &comm::cameraInterface::callbackMsg, camera);
    gazebo::transport::SubscriberPtr lidarSubscriber = node->Subscribe("~/pioneer2dx/hokuyo/link/laser/scan", &comm::lidarInterface::callbackMsg, lidar);

    // Capability to publish to reset the world
    gazebo::transport::PublisherPtr worldPublisher = node->Advertise<gazebo::msgs::WorldControl>("~/world_control");
    gazebo::msgs::WorldControl controlMessage;
    controlMessage.mutable_reset()->set_all(true);
    worldPublisher->WaitForConnection();
    worldPublisher->Publish(controlMessage);

    // TODO Create publisher to publish the location of the closest marbel

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Pre-compute calibration matrixes
    cv::Matx33f K{1047, 0, 0.5*320,
                  0, 1047, 0.5*240,
                  0, 0    , 1};
    cv::Vec<float, 5> k(-0.25, 0.12, -0.00028, 0.00005, 0); // distortion coefficients
    cv::Size frameSize(320, 240);
    cv::Mat mapX, mapY;
    cv::initUndistortRectifyMap(K, k, cv::Matx33f::eye(), K, frameSize, CV_32FC1,
                              mapX, mapY);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Control loop
    while(true)
    {
        // Declare the local variables
        cv::Mat gauss, median, blurred, sharp, laplacianGauss, laplacianBlur, laplacianMedian, laplacian, used, original, undistorted, canny;
        std::vector<cv::Vec3f> circles, circlesLap;

        // Make it capable to read the image 
        mutex.lock();
        int key = cv::waitKey(1);
        mutex.unlock();

        // TODO Conditions to check which key has been pressed to update the adequate values
        if ((key == key_up) && ((param1 + incrementParam) <= maxParam1))
        {
            param1 += incrementParam;
            std::cout << "Incrementing param 1: " << param1 << std::endl;
        }
        if ((key == key_down) && ((param1 - incrementParam) >= minParam1))
        {
            param1 -= incrementParam;
            std::cout << "Reducing param 1: " << param1 << std::endl;
        }
        if (key == key_esc)
        {
            break;
        }

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Initial filtering of the image
        //TODO Implement Rotation Mask

        // Don't do anything if you haven't received an image
        if (!(camera->receptionAccomplished()) | !(lidar->receptionAccomplished())) {continue;}

        // Obtain the image and correct distortion
        original = camera->checkReceived();
        //cv::undistort(original_, original,K,k);
        cv::remap(original, undistorted, mapX, mapY, cv::INTER_LANCZOS4);
        std::cout << "After the undistortion:  " << undistorted.channels() << std::endl;

        // Create copy of original image in grayscale
        cv::cvtColor(undistorted, image_gray, cv::COLOR_BGR2GRAY);

        // Basic filterings to check which one is better
        cv::medianBlur(undistorted, median, kernel_size);
        cv::cvtColor(median, median, cv::COLOR_BGR2GRAY); // For further analysis

        // Edge detection
        median.convertTo(median, CV_32F);
        cv::Laplacian(median, laplacian, CV_32F, 5);
        median = median - 0.1*laplacian;
        median.convertTo(median, CV_8U);
        cv::Canny(median, canny, 400, 450, 5, true);

        // Segmentation/recognition of marbles
        cv::HoughCircles(canny , circles, cv::HOUGH_GRADIENT, 1,
                 canny.rows/16,
                 param1, param2, minRadius, maxRadius 
        );

        std::cout << "Number of circles detected is: " << circles.size() << std::endl;

        // Filtering of the circles
        //pushBeginning<std::vector<cv::Vec3f>>(lastCircles, circles, maxLengthCircleVector);
        pushBeginning<circleAvg>(meanCenter, averageCircle(circles), maxLengthCircleVector);

        // Using the LIDAR to find the marbles in the range
        

        // Control of the robot based on the images


        // Show the pictures

        if (show_image){
            // Draw circles detected in each image
            cv::Mat avgCircleImage = median.clone();

            for (int i = 0; i < circles.size(); i++)
            {
                // circle center
                cv::circle( median, cv::Point(circles[i][0], circles[i][1]), 1, cv::Scalar(0,100,100), 3, cv::LINE_AA);
                // circle outline
                cv::circle( median, cv::Point(circles[i][0], circles[i][1]), circles[i][2], cv::Scalar(255,0,255), 1, cv::LINE_AA);
            }
            cv::circle(avgCircleImage, cv::Point(meanCenter[maxLengthCircleVector-1].x_center, meanCenter[maxLengthCircleVector-1].y_center), meanCenter[maxLengthCircleVector-1].radius, cv::Scalar(255,0,255), 1, cv::LINE_AA);

            /* mutex.lock();
            cv::imshow("Original", original);
            mutex.unlock();
            mutex.lock();
            cv::imshow("Undistorted", undistorted);
            mutex.unlock(); */
            mutex.lock();
            cv::imshow("Median", median);
            mutex.unlock();
            /* mutex.lock();
            cv::imshow("Laplacian", laplacian);
            mutex.unlock(); */
            mutex.lock();
            cv::imshow("Average Circle", avgCircleImage);
            mutex.unlock();
            mutex.lock();
            cv::imshow("Canny edge", canny);
            mutex.unlock();
        }
    }

    // Turn off gazebo
    gazebo::client::shutdown();

    return 0;
}