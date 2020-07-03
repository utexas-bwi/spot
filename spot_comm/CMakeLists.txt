cmake_minimum_required(VERSION 3.0.2)
project(spot_comm)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  nav_msgs
  grpc
)

message("-- Searching for grpc C++")
find_package(grpc QUIET)

catkin_python_setup()

catkin_package()

###########
## Build ##
###########


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

if(grpc_FOUND)
	message("Found grpc! Compiling gRPC communication components.")

	#Note : auth.proto imports header.proto, so need to include
	#here as well to generate the auth proto
	generate_proto(auth_proto GRPC SRC_BASE "." FILES 
		bosdyn/api/auth_service.proto
		bosdyn/api/auth.proto
		bosdyn/api/header.proto
	)

	add_executable(auth_client src/examples/auth_client.cpp)
	add_executable(auth_server src/examples/auth_server.cpp)
	target_link_libraries(auth_client ${catkin_LIBRARIES} auth_proto)
	target_link_libraries(auth_server ${catkin_LIBRARIES} auth_proto)
endif()
