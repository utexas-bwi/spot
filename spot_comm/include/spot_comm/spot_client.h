#ifndef SPOT_CLIENT_H
#define SPOT_CLIENT_H

#include <spot_comm/ssl_robot_command_client.h>
#include <spot_comm/ssl_robot_state_client.h>
#include <spot_comm/ssl_auth_client.h>
#include <spot_comm/ssl_estop_client.h>
#include <spot_comm/ssl_lease_client.h>
#include <spot_comm/ssl_time_sync_client.h>
// .. add more here

class SpotClient {
public:
  SpotClient(const std::string& cert, const std::string& key, const std::string& root, const std::string& server);
  bool sendVelocityCommand(const std::string& frame_name , const double& velX, const double& velY, const double& angular, const int& max_secs);
  bool sendVelocityCommand(const double& velX, const double& velY, const double& angular);
  std::string getAuthToken(const std::string& username, const std::string& password);
  bool startTimeSync(int rounds);

 private:
  AuthClient auth_client;
  RobotCommandClient robot_command_client;
  TimeSyncClient time_sync_client;
};

#endif
