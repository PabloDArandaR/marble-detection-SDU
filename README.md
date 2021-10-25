# Marble detection for SDU's Introduction to Computer Vision course project

The purpose of this project consits on detecting the marbles that appear on the image visualized in the camera of the Gazebo simulation given by [the repository related to the project](https://github.com/jakobwilm/rb-rca5). Appart of the camera, the 2D LIDAR will also be used for detecting the depth towards walls and the marbles that are being followed in that moment.

Basic image processing, such as Gaussian filtering, median filtering, or laplacian for edge detection are used for the marble detection section.

# Dependencies

This project requires the following packages to be installed

Package | Installation guide
------------- | --------------
Gazebo 11 and development tools |[Install](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install)
OpenCV 4.2 | [Install](https://docs.opencv.org/4.5.4/d7/d9f/tutorial_linux_install.html)

