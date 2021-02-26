#ifndef TIMER_H
#define TIMER_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h> 
#include <rosgraph_msgs/Clock.h>
#include <google/protobuf/util/time_util.h>

using google::protobuf::Timestamp;
using google::protobuf::util::TimeUtil; 

class Timer {
    public:
        Timer(ros::NodeHandle &n);
        void clock_cb(const rosgraph_msgs::Clock::ConstPtr& msg);
        void set_start_time_and_dur(ros::Duration duration);
        void stop();
    
    private:
        ros::NodeHandle &nh;
        ros::Publisher twist_pub;
        ros::Subscriber clock_sub;
        ros::Time curr_time;
        ros::Duration cmd_dur;
        ros::Time start_time;

};

#endif