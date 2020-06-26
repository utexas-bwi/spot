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
generate_proto(auth_proto GRPC SRC_BASE "." FILES 
	bosdyn/api/auth_service.proto
	bosdyn/api/auth.proto
	bosdyn/api/header.proto
)

add_executable(auth_client_example src/examples/auth.cpp)
target_link_libraries(auth_client_example ${catkin_LIBRARIES} auth_proto)
