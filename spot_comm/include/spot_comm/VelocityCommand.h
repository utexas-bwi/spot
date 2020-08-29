#ifndef VELOCITY_COMMAND_H
#define VELOCITY_COMMAND_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

class VelocityCommand {
    public:
        VelocityCommand(ros::NodeHandle &n);
        void executeCommand(double xVel, double yVel, double angularVel);
    
    private:
        ros::NodeHandle &nh;
        ros::Publisher pub;

};

#endif