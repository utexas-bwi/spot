#include "spot_comm/VelocityCommand.h"

VelocityCommand::VelocityCommand(ros::NodeHandle &n) : nh(n) {
    pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
}

void VelocityCommand::executeCommand(double xVel, double yVel, double angularVel) {
    geometry_msgs::Twist msg;

    msg.linear.x = xVel;
    msg.linear.y = yVel;
    msg.linear.z = 0;

    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = angularVel;

    pub.publish(msg);
    ROS_INFO("published message x: %f, y: %f, z: %f", msg.linear.x, msg.linear.y, msg.angular.z);
        
    ros::spinOnce();
}
