cmake_minimum_required(VERSION 3.0.2)
project(spot_comm)

add_compile_options(-std=c++11)

set(protobuf_MODULE_COMPATIBLE TRUE)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  nav_msgs
  message_generation
)

find_package(Eigen3 REQUIRED)
find_package(Protobuf CONFIG REQUIRED)
message(STATUS "Using protobuf ${protobuf_VERSION}")

set(_PROTOBUF_LIBPROTOBUF protobuf::libprotobuf)
find_package(gRPC CONFIG REQUIRED)
#set(_REFLECTION gRPC::grpc++_reflection)
set(_GRPC_GRPCPP gRPC::grpc++)
message(STATUS "Using gRPC ${gRPC_VERSION}")

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
  ${Protobuf_INCLUDE_DIRS})


include_directories(${CMAKE_CURRENT_BINARY_DIR})

SET (HEADER include/bosdyn/api/header.pb.cc include/bosdyn/api/header.grpc.pb.cc)
SET (AUTH src/auth_client.cpp include/bosdyn/api/auth.pb.cc include/bosdyn/api/auth.grpc.pb.cc include/bosdyn/api/auth_service.pb.cc include/bosdyn/api/auth_service.grpc.pb.cc)

#Note : auth.proto imports header.proto, so need to include
#here as well to generate the auth proto
#[[generate_proto(header_proto GRPC SRC_BASE "." FILES 
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
)]]



add_executable(auth_client ${HEADER} ${AUTH})

#add_executable(estop_client src/estop_client.cpp)
#[[add_executable(directory_client src/directory_client.cpp)
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
)]]

  target_link_libraries(auth_client ${catkin_LIBRARIES}${Protobuf_LIBRARIES}#${_REFLECTION}
    ${_GRPC_GRPCPP}
    ${_PROTOBUF_LIBPROTOBUF})
 #[[ target_link_libraries(estop_client ${catkin_LIBRARIES})
  target_link_libraries(spotServer ${catkin_LIBRARIES} )
  target_link_libraries(directory_client ${catkin_LIBRARIES})
  target_link_libraries(image_client ${catkin_LIBRARIES})
  target_link_libraries(time_sync_client ${catkin_LIBRARIES})
  target_link_libraries(lease_client ${catkin_LIBRARIES})
  target_link_libraries(log_annotation_client ${catkin_LIBRARIES})
  target_link_libraries(robot_command_client ${catkin_LIBRARIES})
  # target_link_libraries(robot_state_client ${catkin_LIBRARIES} header_proto robot_state_proto)
  target_link_libraries(spotServer ${catkin_LIBRARIES}) ]]

catkin_install_python(PROGRAMS scripts/move_client
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#[[add_executable(vel_cmd src/vel_cmd_node.cpp src/VelocityCommand.cpp)
target_link_libraries(vel_cmd ${catkin_LIBRARIES})

add_executable(gps_translator_node src/gps_translator.cc)
target_link_libraries(gps_translator_node ${catkin_LIBRARIES})
add_dependencies(gps_translator_node spot_comm_generate_messages_cpp)]]

