cmake_minimum_required(VERSION 3.0.2)
project(spot_comm)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  grpc
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


catkin_package()

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

#generate_proto(<proto_target> <proto_path>/<proto_name>.proto)
