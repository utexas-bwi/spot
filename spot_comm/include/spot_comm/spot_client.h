#include <spot_comm/ssl_robot_command_client.h>
#include <spot_comm/ssl_auth_client.h>
// .. add more here

class SpotClient {
public:
  SpotClient(const std::string& cert, const std::string& key, const std::string& root, const std::string& server);
  bool sendVelocityCommand(const std::string& frame_name , const double& velX, const double& velY, const double& angular, const int& max_secs);
  bool sendVelocityCommand(const double& velX, const double& velY, const double& angular);

 private:
  AuthClient auth_client;
  RobotCommandClient robot_command_client;
};

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