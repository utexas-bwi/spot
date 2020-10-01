cmake_minimum_required(VERSION 3.0.2)
project(spot_comm)

add_compile_options(-std=c++11)

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
set(_REFLECTION gRPC::grpc++_reflection)
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
  ${Protobuf_INCLUDE_DIRS}
)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

function(include_files name)
    string(TOUPPER ${name} upperName)
    set (${upperName} include/bosdyn/api/${name}.pb.cc include/bosdyn/api/${name}.grpc.pb.cc PARENT_SCOPE)
endfunction()

function(include_files_service name)
    string(TOUPPER ${name} upperName)
    set (${upperName} include/bosdyn/api/${name}.pb.cc include/bosdyn/api/${name}.grpc.pb.cc include/bosdyn/api/${name}_service.pb.cc include/bosdyn/api/${name}_service.grpc.pb.cc PARENT_SCOPE)
endfunction()


include_files(header)
include_files_service(auth)
include_files_service(estop)
include_files_service(directory)
include_files_service(image)
include_files(geometry)
include_files_service(time_sync)
include_files_service(lease)
include_files_service(license)
include_files_service(log_annotation)
include_files_service(power)
include_files_service(robot_command)
include_files(full_body_command)
include_files(mobility_command)
include_files(basic_command)
include_files(trajectory)
include_files_service(robot_state)
include_files(parameter)

add_executable(auth_client src/auth_client.cpp ${HEADER} ${AUTH})
add_executable(estop_client src/estop_client.cpp ${HEADER} ${ESTOP})
add_executable(directory_client src/directory_client.cpp ${HEADER} ${DIRECTORY})
add_executable(image_client src/image_client.cpp ${HEADER} ${IMAGE} ${GEOMETRY})
add_executable(time_sync_client src/time_sync_client.cpp ${HEADER} ${TIME_SYNC})
add_executable(lease_client src/lease_client.cpp ${HEADER} ${LEASE})
add_executable(log_annotation_client src/log_annotation_client.cpp ${HEADER} ${LOG_ANNOTATION})
add_executable(robot_command_client src/robot_command_client.cpp src/VelocityCommand.cpp ${HEADER} ${ROBOT_COMMAND} ${FULL_BODY_COMMAND} ${MOBILITY_COMMAND} ${BASIC_COMMAND} ${TRAJECTORY} ${LEASE} ${GEOMETRY})
add_executable(robot_state_client src/robot_state_client.cpp ${HEADER} ${ROBOT_STATE} ${PARAMETER} ${GEOMETRY})
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
  ${HEADER}
  ${AUTH}
  ${ESTOP}
  ${DIRECTORY}
  ${LEASE}
  ${LOG_ANNOTATION}
  ${IMAGE}
  ${GEOMETRY}
  ${TIME_SYNC}
  ${POWER}
  ${ROBOT_COMMAND}
  ${FULL_BODY_COMMAND}
  ${MOBILITY_COMMAND}
  ${BASIC_COMMAND}
  ${TRAJECTORY}
  ${ROBOT_STATE}
  ${PARAMETER}
  ${LICENSE}
)

  set(LINK_LIBRARIES 
    ${catkin_LIBRARIES}
    ${Protobuf_LIBRARIES}
    ${_GRPC_GRPCPP}
    ${_PROTOBUF_LIBPROTOBUF}
  )
  target_link_libraries(auth_client ${LINK_LIBRARIES})
  target_link_libraries(estop_client ${LINK_LIBRARIES})
  target_link_libraries(directory_client ${LINK_LIBRARIES})
  target_link_libraries(image_client ${LINK_LIBRARIES})
  target_link_libraries(time_sync_client ${LINK_LIBRARIES})
  target_link_libraries(lease_client ${LINK_LIBRARIES})
  target_link_libraries(log_annotation_client ${LINK_LIBRARIES})
  target_link_libraries(robot_command_client ${LINK_LIBRARIES})
  target_link_libraries(robot_state_client ${LINK_LIBRARIES})
  target_link_libraries(spotServer ${LINK_LIBRARIES} ${_REFLECTION})

catkin_install_python(PROGRAMS scripts/move_client
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#[[add_executable(vel_cmd src/vel_cmd_node.cpp src/VelocityCommand.cpp)
target_link_libraries(vel_cmd ${catkin_LIBRARIES})

add_executable(gps_translator_node src/gps_translator.cc)
target_link_libraries(gps_translator_node ${catkin_LIBRARIES})
add_dependencies(gps_translator_node spot_comm_generate_messages_cpp)]]

