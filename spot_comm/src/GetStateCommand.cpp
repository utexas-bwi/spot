#include "spot_comm/GetStateCommand.h"

// static void modelStateCallback(const gazebo_msgs::ModelStates::ConstPtr &msg) {
//     auto twist = msg->twist;
//     auto index = sizeof(twist)/sizeof(twist[0]) - 1;
//     ROS_INFO("Linear: %f, %f, %f Angular: %f %f %f\n", twist[index].linear.x, twist[index].linear.y, twist[index].linear.z, twist[index].angular.x,
//          twist[index].angular.y, twist[index].angular.z);
//     ros::spinOnce();
// }

// GetStateCommand::GetStateCommand(ros::NodeHandle &n) : nh(n) {
//     sub = nh.subscribe("/gazebo/model_states", 10, &modelStateCallback);
// }
