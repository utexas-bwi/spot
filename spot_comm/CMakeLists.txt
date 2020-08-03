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
  message_generation
  grpc
)

message("-- Searching for grpc C++")
find_package(grpc QUIET)
find_package(Eigen3 REQUIRED)

catkin_python_setup()

# Message generation
add_action_files(DIRECTORY action FILES Waypoint.action)
generate_messages(DEPENDENCIES std_msgs geometry_msgs actionlib_msgs)

catkin_package(INCLUDE_DIRS include)

###########
## Build ##
###########


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
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

  generate_proto(estop_proto GRPC SRC_BASE "." FILES 
	  bosdyn/api/estop_service.proto
	  bosdyn/api/estop.proto
  )

  generate_proto(directory_proto GRPC SRC_BASE "." FILES 
	  bosdyn/api/directory_service.proto
	  bosdyn/api/directory.proto
  )

  generate_proto(image_proto GRPC SRC_BASE "." FILES 
	  bosdyn/api/image_service.proto
    bosdyn/api/image.proto
    bosdyn/api/geometry.proto
  )

  generate_proto(time_sync_proto GRPC SRC_BASE "." FILES 
	  bosdyn/api/time_sync_service.proto
    bosdyn/api/time_sync.proto
  )

  add_executable(auth_client src/auth_client.cpp)
  add_executable(directory_client src/directory_client.cpp)
  add_executable(spotServer
    src/server.cpp
    src/AuthServiceImpl.cpp
    src/DirectoryServiceImpl.cpp
    src/EstopServiceImpl.cpp
    src/ImageServiceImpl.cpp
    src/TimeSyncServiceImpl.cpp
  )

  target_link_libraries(directory_client ${catkin_LIBRARIES} directory_proto)
  target_link_libraries(auth_client ${catkin_LIBRARIES} auth_proto)
  target_link_libraries(spotServer ${catkin_LIBRARIES} auth_proto directory_proto estop_proto image_proto time_sync_proto)
endif()

catkin_install_python(PROGRAMS scripts/move_client
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(gps_translator_node src/gps_translator.cc)
target_link_libraries(gps_translator_node ${catkin_LIBRARIES})
add_dependencies(gps_translator_node spot_comm_generate_messages_cpp)
