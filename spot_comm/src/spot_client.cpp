#include <spot_comm/spot_client.h>

SpotClient::SpotClient(const std::string& cert, const std::string& key, const std::string& root, const std::string& server) : 
    auth_client(cert, key, root, server), robot_command_client(cert, key, root, server), 
    time_sync_client(cert, key, root, server) {}

bool SpotClient::sendVelocityCommand(const std::string& frame_name, const double& velX, const double& velY, const double& angular, const int& max_secs) {
    // test lease, must get using LeaseClient to actually work
    Lease lease;
    lease.set_resource("testResource");
    lease.set_epoch("testEpoch");
    lease.add_sequence(100);

    RobotCommand command;
    Duration commandDuration;
    commandDuration.set_seconds(max_secs);
    Timestamp end = TimeUtil::GetCurrentTime() + commandDuration;
    command.mutable_mobility_command()->mutable_se2_velocity_request()->mutable_end_time()->CopyFrom(end);
    command.mutable_mobility_command()->mutable_se2_velocity_request()->set_se2_frame_name(frame_name);
    command.mutable_mobility_command()->mutable_se2_velocity_request()->mutable_velocity()->mutable_linear()->set_x(velX);
    command.mutable_mobility_command()->mutable_se2_velocity_request()->mutable_velocity()->mutable_linear()->set_y(velY);
    command.mutable_mobility_command()->mutable_se2_velocity_request()->mutable_velocity()->set_angular(angular);

    RobotCommandResponse response = robot_command_client.startRobotCommand(lease, command);
    return response.status() == RobotCommandResponse::STATUS_OK;
}

bool SpotClient::sendVelocityCommand(const double& velX, const double& velY, const double& angular) {
    return SpotClient::sendVelocityCommand("odom", velX, velY, angular, 10);
}

std::string SpotClient::getAuthToken(const std::string& username, const std::string& password) {
    return auth_client.GetAuthToken(username, password);
}

bool SpotClient::startTimeSync(int rounds) {
    TimeSyncUpdateResponse response = time_sync_client.EstablishTimeSync(rounds);
    return response.state().status() == TimeSyncState::STATUS_OK;
}
