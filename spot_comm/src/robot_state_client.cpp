#include <iostream>
#include <memory>
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>

#include <grpc++/grpc++.h>
#include <grpc++/health_check_service_interface.h>
#include <grpc++/ext/proto_server_reflection_plugin.h>

#include "bosdyn/api/robot_state_service.grpc.pb.h"
#include "bosdyn/api/geometry.grpc.pb.h"
#include <google/protobuf/util/time_util.h>
#include "bosdyn/api/header.grpc.pb.h"
#include <ros/ros.h>
#include <ros/package.h>

using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;
using bosdyn::api::RobotStateService;
using bosdyn::api::RobotStateRequest;
using bosdyn::api::RobotStateResponse;
using bosdyn::api::RobotState;
using bosdyn::api::PowerState;
using bosdyn::api::CommsState;
using bosdyn::api::SystemFaultState;
using bosdyn::api::EStopState;
using bosdyn::api::KinematicState;
using bosdyn::api::BehaviorFaultState; 
using bosdyn::api::FootState;
using bosdyn::api::RobotMetricsRequest;
using bosdyn::api::RobotMetricsResponse;
using bosdyn::api::RobotHardwareConfigurationRequest;
using bosdyn::api::RobotHardwareConfigurationResponse;
using bosdyn::api::RobotLinkModelRequest;
using bosdyn::api::RobotLinkModelResponse;
using google::protobuf::Duration;
using google::protobuf::Timestamp;
using google::protobuf::util::TimeUtil;

class RobotStateClient {
 public:
  RobotStateClient(const std::string& cert, const std::string& key, const std::string& root, const std::string& server) {
    grpc::SslCredentialsOptions opts = {root, key, cert};
    stub_ = RobotStateService::NewStub(grpc::CreateChannel(server, grpc::SslCredentials(opts)));
  } 

  // Assembles the client's payload, sends it and presents the response back
  // from the server.
  RobotStateResponse GetRobotState() {
    // Data we are sending to the server.
    RobotStateRequest request;
    request.mutable_header()->mutable_request_timestamp()->CopyFrom(TimeUtil::GetCurrentTime());
    
    
    // Container for the data we expect from the server.
    RobotStateResponse reply;

    // Context for the client. It could be used to convey extra information to
    // the server and/or tweak certain RPC behaviors.
    ClientContext context;

    // The actual RPC.
    Status status = stub_->GetRobotState(&context, request, &reply);

    // Act upon its status.
    if (status.ok()) {
      // std::cout << "Command status: " << reply.status() << ", Token: " << reply.token() << std::endl;
      std::cout << "Success" << std::endl;
    //   std::cout << reply.message() << std::endl;
      // return "reply.token()";
    } else {
      std::cout << status.error_code() << ": " << status.error_message()
                << std::endl;
      // return "RPC failed";
    }

    return reply;
  }

 private:
  std::unique_ptr<RobotStateService::Stub> stub_;
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


int main(int argc, char** argv) {
  // Instantiate the client. It requires a channel, out of which the actual RPCs
  // are created. This channel models a connection to an endpoint specified by
  // the argument "--target=" which is the only expected argument.
  // We indicate that the channel isn't authenticated (use of
  // InsecureChannelCredentials()).
  std::string server {"localhost:50051"};

  std::string pathToPackage = ros::package::getPath("spot_comm");
  std::string key, cert, root;
  read(pathToPackage + "/include/certs/client.key", key);
  read(pathToPackage + "/include/certs/client.crt", cert);
  read(pathToPackage + "/include/certs/ca.crt", root);

  RobotStateClient robotClient(cert, key, root, server);  
  RobotStateResponse reply = robotClient.GetRobotState();
  std::cout << "Kinematic state timestamp: " << reply.robot_state().kinematic_state().acquisition_timestamp() << std::endl;
  std::cout << "Linear velocity vision x: " << reply.robot_state().kinematic_state().velocity_of_body_in_vision().linear().x() << std::endl;
  std::cout << "Linear velocity vision y: " << reply.robot_state().kinematic_state().velocity_of_body_in_vision().linear().y() << std::endl;
  std::cout << "Linear velocity vision z: " << reply.robot_state().kinematic_state().velocity_of_body_in_vision().linear().z() << std::endl;
  std::cout << "Angular velocity vision x: " << reply.robot_state().kinematic_state().velocity_of_body_in_vision().angular().x() << std::endl;
  std::cout << "Angular velocity vision y: " << reply.robot_state().kinematic_state().velocity_of_body_in_vision().angular().y() << std::endl; 
  std::cout << "Angular velocity vision z: " << reply.robot_state().kinematic_state().velocity_of_body_in_vision().angular().z() << std::endl;
  // std::cout << "Linear velocity odom x: " << reply.robot_state().kinematic_state().velocity_of_body_in_odom().linear().x() << std::endl;
  // std::cout << "Angular velocity odom z: " << reply.robot_state().kinematic_state().velocity_of_body_in_odom().angular().z() << std::endl;

  return 0;
}