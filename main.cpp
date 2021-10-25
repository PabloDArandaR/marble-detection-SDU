#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdlib.h>  // for strtol

#include "src/communication.cpp"

static boost::mutex mutex;
const int key_left = 81;
const int key_up = 82;
const int key_down = 84;
const int key_right = 83;
const int key_esc = 27;

const int maxDev {5};
const int minDev {0};
const double incrementDev {0.02};
const int param1 {100};
const int param2 {40};
const int maxRadius {200};
const int minRadius {50};

int main(int argc, char ** argv)
{
    // Declaration of global variables
    double stdDev {0.02};
    const double mean {0};
    const int kernel_size {3};
    bool show_image {false};
    cv::Mat image, image_gray, filtered;
    comm::cameraInterface * camera (new comm::cameraInterface);

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
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    // Listen to Gazebo topics
    gazebo::transport::SubscriberPtr cameraSubscriber = node->Subscribe("~/pioneer2dx/camera/link/camera/image", &comm::cameraInterface::cameraCallback, camera);

    // Capability to publish to reset the world
    gazebo::transport::PublisherPtr worldPublisher = node->Advertise<gazebo::msgs::WorldControl>("~/world_control");
    gazebo::msgs::WorldControl controlMessage;
    controlMessage.mutable_reset()->set_all(true);
    worldPublisher->WaitForConnection();
    worldPublisher->Publish(controlMessage);

    // TODO Create publisher to publish the location of the closest marbel

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Control loop
    while(true)
    {
        // Declare the local variables
        cv::Mat gauss, median, blurred, sharp, laplacianGauss, laplacianBlur, laplacianMedian, laplacian, used, original;
        std::vector<cv::Vec3f> circles;

        // Make it capable to read the image 
        mutex.lock();
        int key = cv::waitKey(1);
        mutex.unlock();

        // TODO Conditions to check which key has been pressed to update the adequate values
        if ((key == key_up) && ((stdDev + incrementDev) <= maxDev))
        {
            stdDev += incrementDev;
            std::cout << "Incrementing standard deviation: " << stdDev << std::endl;
        }
        if ((key == key_down) && ((stdDev - incrementDev) >= minDev))
        {
            stdDev -= incrementDev;
            std::cout << "Reducing standard deviation: " << stdDev << std::endl;
        }
        if (key == key_esc)
        {
            break;
        }

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Initial filtering of the image
        //TODO Implement Rotation Mask

        // Don't do anything if you haven't received an image
        if (!(camera->receptionAccomplished())) {continue;}

        // Obtain the image and convert it to grayscale
        original = camera->checkImage();
        cv::cvtColor(original, image_gray, cv::COLOR_BGR2GRAY);

        // Basic filterings to check which one is better
        used = original;
        cv::GaussianBlur(used, filtered, cv::Size(kernel_size, kernel_size), stdDev);
        cv::blur(used, blurred, cv::Size(kernel_size, kernel_size));
        cv::medianBlur(filtered, median, kernel_size);
        cv::GaussianBlur(image_gray, image_gray, cv::Size(kernel_size, kernel_size), stdDev);
        //cv::blur(image_gray, image_gray, cv::Size(kernel_size, kernel_size));
        //cv::medianBlur(image_gray, image_gray, kernel_size);

        // First derivative use
        /* cv::Mat horizontal, vertical;
        cv::Sobel(image_gray, horizontal, CV_32F,1,0);
        cv::Sobel(image_gray, vertical, CV_32F,0,1);
        vertical.convertTo(vertical, CV_8U);
        horizontal.convertTo(horizontal, CV_8U);
        cv::Mat sum = horizontal + vertical; */

        // Second derivative use
        /*image_gray.convertTo(image_gray, CV_32F);
        cv::Laplacian(image_gray, laplacian, CV_32F, 3);
        image_gray -= 0.3*laplacian;
        image_gray.convertTo(image_gray, CV_8U); */

        // Segmentation/recognition of marbles
        
        cv::HoughCircles(image_gray, circles, cv::HOUGH_GRADIENT, 1,
                 image_gray.rows/16,
                 param1, param2, minRadius, maxRadius 
        );

        // Control of the robot based on the images


        // Show the pictures

        if (show_image){
            // Draw circles detected in each image
            for (int i = 0; i < circles.size(); i++)
            {
                // circle center
                cv::circle( image_gray, cv::Point(circles[i][0], circles[i][1]), 1, cv::Scalar(0,100,100), 3, cv::LINE_AA);
                // circle outline
                circle( image_gray, cv::Point(circles[i][0], circles[i][1]), circles[i][2], cv::Scalar(255,0,255), 3, cv::LINE_AA);
            }

            mutex.lock();
            cv::imshow("Show gray image", image_gray);
            mutex.unlock();
        }
    }

    // Turn off gazebo
    gazebo::client::shutdown();

    return 0;
}