#include <spot_comm/RobotStateServiceImpl.h>
#include <spot_comm/Header.h>

Status RobotStateServiceImpl::GetRobotState(ServerContext* context, const RobotStateRequest* request, RobotStateResponse* response) {
    // header
    response->mutable_header()->CopyFrom(Header::generateResponseHeader(request->header()));

    // power state

    // battery state

    // comms state

    // system fault state

    // estop state

    // kinematic state - seems to be most important
    response->mutable_robot_state()->mutable_kinematic_state()->mutable_acquisition_timestamp()->CopyFrom(TimeUtil::GetCurrentTime());
    // response->mutable_robot_state()->mutable_kinematic_state()->mutable_joint_states(); // Get from ROS JointStatePublisher
    // response->mutable_robot_state()->mutable_kinematic_state()->mutable_transforms_snapshot(); // Get from TF tree
    
    // Get velocities from ROS publishers
    response->mutable_robot_state()->mutable_kinematic_state()->mutable_velocity_of_body_in_vision()->mutable_linear()->set_x(1.0);
    response->mutable_robot_state()->mutable_kinematic_state()->mutable_velocity_of_body_in_vision()->mutable_linear()->set_y(0.0); 
    response->mutable_robot_state()->mutable_kinematic_state()->mutable_velocity_of_body_in_vision()->mutable_angular()->set_z(0.0);
    response->mutable_robot_state()->mutable_kinematic_state()->mutable_velocity_of_body_in_odom()->mutable_linear()->set_x(3.0);
    response->mutable_robot_state()->mutable_kinematic_state()->mutable_velocity_of_body_in_odom()->mutable_linear()->set_y(0.0); 
    response->mutable_robot_state()->mutable_kinematic_state()->mutable_velocity_of_body_in_odom()->mutable_angular()->set_z(0.0);

    // behavior fault state

    // foot state
    
    return Status::OK;
}

Status RobotStateServiceImpl::GetRobotMetrics(ServerContext* context, const RobotMetricsRequest* request, RobotMetricsResponse* response) {
    // header
    response->mutable_header()->CopyFrom(Header::generateResponseHeader(request->header()));
    
    return Status::OK;
}

Status RobotStateServiceImpl::GetRobotHardwareConfiguration(ServerContext* context, const RobotHardwareConfigurationRequest* request, RobotHardwareConfigurationResponse* response) {
    // header
    response->mutable_header()->CopyFrom(Header::generateResponseHeader(request->header()));
    
    return Status::OK;
}

Status RobotStateServiceImpl::GetRobotLinkModel(ServerContext* context, const RobotLinkModelRequest* request, RobotLinkModelResponse* response) {
    // header
    response->mutable_header()->CopyFrom(Header::generateResponseHeader(request->header()));
    
    return Status::OK;
}

