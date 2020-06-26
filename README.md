# The SPOT Repository, as part of the SMADS Project

This repository hosts Gazebo simulation files for the Boston Dynamics Spot, as well as the software interfaces that allow communication with Spot, via gRPC, ove ROS.

## Installation
Clone this repository into a catkinized workspace:

	$ git clone --recurse-submodules https://github.com/utexas-bwi/spot.git

note that we do utilize other repositories as submodules, so ensure those have been correctly fetched : 

	$ cd spot/grpc && ls


## Compiling
Compile with the standard catkin-tools build system:

	$ catkin build spot


 
