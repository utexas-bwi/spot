#include <ros/ros.h>
#include "spot_comm/VelocityCommand.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "vel_cmd_node");
  ros::NodeHandle n;

  VelocityCommand vel(n);
  
  double xVel;
  double yVel;
  double angularVel;

  int counter = 0;

  ros::Rate r(1000);

  while(ros::ok() && counter < 2000) {
    xVel = 1;
    yVel = 1;
    angularVel = 0;

    vel.executeCommand(xVel, yVel, angularVel, TimeUtil::GetCurrentTime());

    counter++;
    
    if(counter == 2000) {
      vel.executeCommand(0, 0, 0, TimeUtil::GetCurrentTime());
    }
    
    r.sleep();
    ros::spinOnce();
  }

  return 0;
}
