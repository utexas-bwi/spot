#ifndef SPOT_DRIVER_H
#define SPOT_DRIVER_H

#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Twist.h>
#include <spot_comm/spot_client.h>

void read(const std::string& filename, std::string& data) {
  std::ifstream file(filename.c_str(), std::ios::in);
  if (file.is_open()) {
    std::stringstream ss;
    ss << file.rdbuf();
    file.close();
    data = ss.str();
  }
  return;
}

SpotClient* spot_client = nullptr;


#endif