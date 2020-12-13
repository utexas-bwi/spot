#include "spot_comm/VelocityCommand.h"

VelocityCommand::VelocityCommand(ros::NodeHandle &n) : nh(n), timer(Timer(n)) {
    pub = nh.advertise<geometry_msgs::Twist>("spot_sim/cmd_vel", 1000);
    twist_sub = nh.subscribe("spot_sim/cmd_vel", 1, &VelocityCommand::twist_cb, this);
}

void VelocityCommand::twist_cb(const geometry_msgs::Twist::ConstPtr& msg) {
    timer.set_start_time_and_dur(ros::Duration(end.seconds(), end.nanos()));
}

void VelocityCommand::executeCommand(double xVel, double yVel, double angularVel, Timestamp end_time) {
    end = end_time - TimeUtil::GetCurrentTime();
    geometry_msgs::Twist msg;

    msg.linear.x = xVel;
    msg.linear.y = yVel;
    msg.linear.z = 0;

    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = angularVel;

    pub.publish(msg);
    ROS_INFO("published message x: %f, y: %f, z: %f", msg.linear.x, msg.linear.y, msg.angular.z);
}
