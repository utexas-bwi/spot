cmake_minimum_required(VERSION 3.0.2)
project(spot_comm)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  grpc
)

catkin_package(INCLUDE_DIRS include)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


#Note : auth.proto imports header.proto, so need to include
#here as well to generate the auth proto
generate_proto(auth_proto GRPC SRC_BASE "." FILES 
	bosdyn/api/auth_service.proto
	bosdyn/api/auth.proto
	bosdyn/api/header.proto
)

generate_proto(estop_proto GRPC SRC_BASE "." FILES 
	bosdyn/api/estop_service.proto
	bosdyn/api/estop.proto
)

add_executable(auth_client src/auth_client.cpp)
add_executable(spotServer
  src/server.cpp
  src/AuthServiceImpl.cpp
)

target_link_libraries(auth_client ${catkin_LIBRARIES} auth_proto)
target_link_libraries(spotServer ${catkin_LIBRARIES} auth_proto estop_proto)

catkin_install_python(PROGRAMS scripts/move_client
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
