cmake_minimum_required(VERSION 3.0.2)
project(spot_comm)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  grpc
)

catkin_package()

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

#Note : auth.proto imports header.proto, so need to include
#here as well to generate the auth proto
generate_proto(auth GRPC SRC_BASE "." FILES 
	api/auth.proto
	api/header.proto
)

