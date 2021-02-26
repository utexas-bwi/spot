#ifndef VELOCITY_COMMAND_H
#define VELOCITY_COMMAND_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <google/protobuf/util/time_util.h>
#include <spot_comm/Timer.h>

using google::protobuf::Duration;
using google::protobuf::Timestamp;
using google::protobuf::util::TimeUtil; 

class VelocityCommand {
    public:
        VelocityCommand(ros::NodeHandle &n);
        void executeCommand(double xVel, double yVel, double angularVel, Timestamp end);
        void twist_cb(const geometry_msgs::Twist::ConstPtr& msg);
    
    private:
        ros::NodeHandle &nh;
        ros::Publisher pub;
        ros::Subscriber twist_sub;
        Timer timer;
        Duration end;

};

#endif
