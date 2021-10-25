# Marble detection for SDU's Introduction to Computer Vision course project

The purpose of this project consits on detecting the marbles that appear on the image visualized in the camera of the Gazebo simulation given by [the repository related to the project](https://github.com/jakobwilm/rb-rca5). Appart of the camera, the 2D LIDAR will also be used for detecting the depth towards walls and the marbles that are being followed in that moment.

Basic image processing, such as Gaussian filtering, median filtering, or laplacian for edge detection are used for the marble detection section.

# Dependencies

This project requires the following packages to be installed

Package | Installation guide
------------- | --------------
Gazebo 11 and development tools |[Install](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install)
OpenCV 4.2 | [Install](https://docs.opencv.org/4.5.4/d7/d9f/tutorial_linux_install.html)

# Instructions

The first thing you will need to do is build the package. The conventional way to do this is:
1. Clone the repository: ``` git clone git@github.com:PabloDavidAR/marble-detection-SDU.git```
2. Go to the repository folder: ``` cd marble-detection-SDU```
3. Create a build folder and go to it: ``` mkdir build && cd build```
4. Generate the makefiles: ``` cmake ..```
5. Build the package: ```make```

Once this is done, you can either use the package with your own project (you will need to modify the name of the node that this package is communicating with) or use it in combination with the package that this project depended on.

1. Since this package is based in the [previously mentioned repository](https://github.com/jakobwilm/rb-rca5), to make this package work you would need to first clone that repository.
2. Run ```./gazebo_server.sh nameOfTheWorld.world```, where ```nameOfTheWorld``` referes to the name of the file that contains the world to be ran.
3. Go to the build folder of the package: ```cd /path/to/marble-detection-SDU/build```
4. Execute the command with the desired flags and inputs: ```./robot_control```

# Recommendations
When using this package for your own projects, remember that some of the implementations used, such as the pattern matching implementation is dependant on the image used as template. You will need to use you own image if you intend to use this feature.
