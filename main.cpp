#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <opencv2/opencv.hpp>
#include <iostream>

#include "src/filter.hpp"

static boost::mutex mutex;
const int key_left = 81;
const int key_up = 82;
const int key_down = 84;
const int key_right = 83;
const int key_esc = 27;

int main(int argc, char ** argv)
{
    // Declaration of global variables
    const double stdDev {0.02};
    const double mean {0};
    const int kernel_size {3};
    cv::Mat image, image_gray, filtered;
    comm::cameraInterface camera;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Setup Gazebo and ROS Subscriptions and Publishers

    // Load Gazebo's client
    gazebo::client::setup(argc, argv);

    // Creation of node for communication
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    // Listen to Gazebo topics
    gazebo::transport::SubscriberPtr cameraSubscriber = node->Subscribe("~/pioneer2dx/camera/link/camera/image", comm::cameraInterface::cameraCallback, camera);

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
        cv::Mat gauss, median, blurred, sharp, laplacian, used;
        // Make it capable to read the image 
        mutex.lock();
        int key = cv::waitKey(1);
        mutex.unlock();

        // TODO Conditions to check which key has been pressed to update the adequate values

        // Initial filtering of the image
        //TODO Implement Rotation Mask
        cv::cvtColor(camera.checkImage(), image_gray, cv::COLOR_RGB2GRAY);
        used = camera.checkImage();
        cv::GaussianBlur(used, filtered, cv::Size(kernel_size, kernel_size), stdDev);
        cv::blur(used, blurred, cv::Size(kernel_size, kernel_size), stdDev);
        cv::medianBlur(used, median, kernel_size);

        // Segmentation/recognition

        // Control of the robot based on the images

    }

    return 0;
}