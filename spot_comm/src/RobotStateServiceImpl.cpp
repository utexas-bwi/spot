#include <spot_comm/RobotStateServiceImpl.h>
#include <spot_comm/Header.h>
#include <ros/ros.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h> 

RobotStateServiceImpl::RobotStateServiceImpl(ros::NodeHandle &n): nh(n) {}



Status RobotStateServiceImpl::GetRobotState(ServerContext* context, const RobotStateRequest* request, RobotStateResponse* response) {
    response->mutable_header()->CopyFrom(Header::generateResponseHeader(request->header()));

    response->mutable_robot_state()->mutable_kinematic_state()->mutable_acquisition_timestamp()->CopyFrom(TimeUtil::GetCurrentTime());

    ros::ServiceClient client = nh.serviceClient<gazebo_msgs::GetModelState>("gazebo/get_model_state");
    gazebo_msgs::GetModelState getmodelstate;
    getmodelstate.request.model_name = "spot";
    geometry_msgs::Twist twist;
    if (client.call(getmodelstate)) {
        twist = getmodelstate.response.twist;
        response->mutable_robot_state()->mutable_kinematic_state()->mutable_velocity_of_body_in_vision()->mutable_linear()->set_x(twist.linear.x);
        response->mutable_robot_state()->mutable_kinematic_state()->mutable_velocity_of_body_in_vision()->mutable_linear()->set_y(twist.linear.y);
        response->mutable_robot_state()->mutable_kinematic_state()->mutable_velocity_of_body_in_vision()->mutable_linear()->set_z(twist.linear.z);
        response->mutable_robot_state()->mutable_kinematic_state()->mutable_velocity_of_body_in_vision()->mutable_angular()->set_x(twist.angular.x);
        response->mutable_robot_state()->mutable_kinematic_state()->mutable_velocity_of_body_in_vision()->mutable_angular()->set_y(twist.angular.y);        
        response->mutable_robot_state()->mutable_kinematic_state()->mutable_velocity_of_body_in_vision()->mutable_angular()->set_z(twist.angular.z);
        // response->mutable_robot_state()->mutable_kinematic_state()->mutable_velocity_of_body_in_odom()->mutable_linear()->set_x();
        // response->mutable_robot_state()->mutable_kinematic_state()->mutable_velocity_of_body_in_odom()->mutable_linear()->set_y(0.0); 
        // response->mutable_robot_state()->mutable_kinematic_state()->mutable_velocity_of_body_in_odom()->mutable_angular()->set_z(0.0);
    }
    else {
        ROS_INFO("Not able to receive model state.");
    }


    // power state

    // battery state

    // comms state

    // system fault state

    // estop state

    // kinematic state - seems to be most important
    // response->mutable_robot_state()->mutable_kinematic_state()->mutable_joint_states(); // Get from ROS JointStatePublisher
    // response->mutable_robot_state()->mutable_kinematic_state()->mutable_transforms_snapshot(); // Get from TF tree

    
    // Get velocities from ROS publishers

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

