#include <spot_comm/spot_driver.h>

void twist_cb(const geometry_msgs::Twist::ConstPtr& msg) {
  assert(spot_client);
  spot_client->sendVelocityCommand(msg->linear.x, msg->linear.y, msg->angular.z);
}

int main(int argc, char** argv) {
  assert(argc >= 4);
  // put driver code here, use spot_client
  std::string server = argv[1];
  std::string username = argv[2];
  std::string password = argv[3];
  
  
  // need to switch between sim CA cert and real cert, rn just sim cert
  std::string pathToPackage = ros::package::getPath("spot_comm");
  std::string key, cert, root;
  read(pathToPackage + "/include/certs/client.key", key);
  read(pathToPackage + "/include/certs/client.crt", cert);
  read(pathToPackage + "/include/certs/ca.crt", root);

  spot_client = new SpotClient(cert, key, root, server);
  
  ros::init(argc, argv, "spot_driver");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("cmd_vel", 1000, twist_cb);

  ros::spin();
  
  return 0;
}