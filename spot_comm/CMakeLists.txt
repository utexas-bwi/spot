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

  generate_proto(header_proto GRPC SRC_BASE "." FILES 
    bosdyn/api/header.proto
  )

  generate_proto(auth_proto GRPC SRC_BASE "." FILES 
    bosdyn/api/auth_service.proto
    bosdyn/api/auth.proto
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

  generate_proto(lease_proto GRPC SRC_BASE "." FILES 
    bosdyn/api/lease_service.proto
    bosdyn/api/lease.proto
  )

  generate_proto(license_proto GRPC SRC_BASE "." FILES 
    bosdyn/api/license_service.proto
    bosdyn/api/license.proto
  )

  generate_proto(log_annotation_proto GRPC SRC_BASE "." FILES 
    bosdyn/api/log_annotation_service.proto
    bosdyn/api/log_annotation.proto
  )

  generate_proto(power_proto GRPC SRC_BASE "." FILES 
    bosdyn/api/power_service.proto
    bosdyn/api/power.proto
  )

  generate_proto(robot_command_proto GRPC SRC_BASE "." FILES
    bosdyn/api/robot_command_service.proto
    bosdyn/api/robot_command.proto
    bosdyn/api/full_body_command.proto
    bosdyn/api/mobility_command.proto
    bosdyn/api/basic_command.proto
    bosdyn/api/trajectory.proto
  )

  generate_proto(robot_state_proto GRPC SRC_BASE "." FILES
    bosdyn/api/robot_state.proto
    bosdyn/api/robot_state_service.proto
    bosdyn/api/parameter.proto
  )

  add_executable(auth_client src/auth_client.cpp)
  add_executable(estop_client src/estop_client.cpp)
  add_executable(directory_client src/directory_client.cpp)
  add_executable(image_client src/image_client.cpp)
  add_executable(time_sync_client src/time_sync_client.cpp)
  add_executable(lease_client src/lease_client.cpp)
  add_executable(log_annotation_client src/log_annotation_client.cpp)
  add_executable(robot_command_client src/robot_command_client.cpp src/VelocityCommand.cpp)
  add_executable(robot_state_client src/robot_state_client.cpp)
  add_executable(spotServer
    src/server.cpp
    src/AuthServiceImpl.cpp
    src/DirectoryServiceImpl.cpp
    src/LeaseServiceImpl.cpp
    src/LogAnnotationServiceImpl.cpp
    src/EstopServiceImpl.cpp
    src/ImageServiceImpl.cpp
    src/TimeSyncServiceImpl.cpp
    src/PowerServiceImpl.cpp
    src/RobotCommandServiceImpl.cpp
    src/RobotStateServiceImpl.cpp
  )

  target_link_libraries(auth_client ${catkin_LIBRARIES} auth_proto header_proto)
  target_link_libraries(estop_client ${catkin_LIBRARIES} estop_proto header_proto)
  target_link_libraries(spotServer ${catkin_LIBRARIES} auth_proto estop_proto image_proto header_proto)
  target_link_libraries(directory_client ${catkin_LIBRARIES} header_proto directory_proto)
  target_link_libraries(image_client ${catkin_LIBRARIES} header_proto image_proto)
  target_link_libraries(time_sync_client ${catkin_LIBRARIES} header_proto time_sync_proto)
  target_link_libraries(lease_client ${catkin_LIBRARIES} header_proto lease_proto)
  target_link_libraries(log_annotation_client ${catkin_LIBRARIES} header_proto log_annotation_proto)
  target_link_libraries(robot_command_client ${catkin_LIBRARIES} header_proto image_proto lease_proto robot_command_proto)
  target_link_libraries(robot_state_client ${catkin_LIBRARIES} header_proto image_proto robot_state_proto)
  target_link_libraries(spotServer ${catkin_LIBRARIES} auth_proto directory_proto estop_proto header_proto image_proto power_proto robot_command_proto robot_state_proto time_sync_proto lease_proto license_proto log_annotation_proto)
endif()

catkin_install_python(PROGRAMS scripts/move_client
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(vel_cmd src/vel_cmd_node.cpp src/VelocityCommand.cpp)
target_link_libraries(vel_cmd ${catkin_LIBRARIES})

add_executable(gps_translator_node src/gps_translator.cc)
target_link_libraries(gps_translator_node ${catkin_LIBRARIES})
add_dependencies(gps_translator_node spot_comm_generate_messages_cpp)
