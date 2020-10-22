#include <spot_comm/RobotStateServiceImpl.h>
#include <spot_comm/Header.h>
#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h> 
#include <spot_comm/GetStateCommand.h>

RobotStateServiceImpl::RobotStateServiceImpl(ros::NodeHandle &n): nh(n) {}

static void modelStateCallback(const gazebo_msgs::ModelStates::ConstPtr &msg) {
    auto twist = msg->twist;
    auto index = sizeof(twist)/sizeof(twist[0]) - 1;
    ROS_INFO("Linear: %f, %f, %f Angular: %f %f %f\n", twist[index].linear.x, twist[index].linear.y, twist[index].linear.z, twist[index].angular.x,
         twist[index].angular.y, twist[index].angular.z);
    ros::spinOnce();
}

Status RobotStateServiceImpl::GetRobotState(ServerContext* context, const RobotStateRequest* request, RobotStateResponse* response) {
    ros::Subscriber sub = nh.subscribe("/gazebo/model_states", 10, &modelStateCallback);
    ros::spinOnce();
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

