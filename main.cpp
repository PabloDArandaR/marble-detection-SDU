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
        std::cout << "value of input: " << show_image << std::endl;
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
        cv::Mat gauss, median, blurred, sharp, laplacian, used, original;

        // Make it capable to read the image 
        mutex.lock();
        int key = cv::waitKey(1);
        mutex.unlock();

        // TODO Conditions to check which key has been pressed to update the adequate values
        if ((key == key_up) && ((stdDev + incrementDev) <= maxDev))
        {
            stdDev += incrementDev;
        }
        if ((key == key_down) && ((stdDev - incrementDev) <= minDev))
        {
            stdDev -= incrementDev;
        }
        if (key == key_esc)
        {
            break;
        }

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Initial filtering of the image
        //TODO Implement Rotation Mask

        // Don't do anything if you haven't received an image
        if (camera->receptionAccomplished()) {continue;}

        // Obtain the image and convert it to grayscale
        original = camera->checkImage();
        cv::cvtColor(original, image_gray, cv::COLOR_RGB2GRAY);

        // Basic filterings to check which one is better
        used = original;
        cv::GaussianBlur(used, filtered, cv::Size(kernel_size, kernel_size), stdDev);
        cv::blur(used, blurred, cv::Size(kernel_size, kernel_size));
        cv::medianBlur(used, median, kernel_size);

        // Segmentation/recognition



        // Control of the robot based on the images

        // Show the pictures
        std::cout << "Inside here \n";
        mutex.lock();
        cv::imshow("Original image.", original);
        mutex.unlock();
        mutex.lock();
        cv::imshow("Gaussian blur", filtered);
        mutex.unlock();
        mutex.lock();
        cv::imshow("Median blur", median);
        mutex.unlock();
    }

    // Turn off gazebo
    gazebo::client::shutdown();

    return 0;
}