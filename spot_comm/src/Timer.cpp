#include "spot_comm/Timer.h"

Timer::Timer(ros::NodeHandle &n) : nh(n) {
    twist_pub = nh.advertise<geometry_msgs::Twist>("spot_sim/cmd_vel", 1000);
    clock_sub = nh.subscribe("/clock", 1, &Timer::clock_cb, this);
    start_time.sec = -1;
}

void Timer::set_start_time_and_dur(ros::Duration duration) {
    start_time = curr_time;
    cmd_dur = duration;
}

void Timer::clock_cb(const rosgraph_msgs::Clock::ConstPtr& msg) {
    curr_time = msg->clock;
    if(start_time.sec != -1 && curr_time - start_time > cmd_dur) {
        stop();
    }
}

void Timer::stop() {
    geometry_msgs::Twist msg;

    msg.linear.x = 0;
    msg.linear.y = 0;
    msg.linear.z = 0;

    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = 0;

    twist_pub.publish(msg);
}