
#include <iostream>
#include <sstream>
#include <fstream>
#include <memory>
#include <string>

#include <grpc++/grpc++.h>
#include <grpc++/health_check_service_interface.h>
#include <grpc++/ext/proto_server_reflection_plugin.h>

#include <spot_comm/AuthServiceImpl.h>
#include <spot_comm/EstopServiceImpl.h>
#include <spot_comm/DirectoryServiceImpl.h>
#include <spot_comm/ImageServiceImpl.h>
#include <spot_comm/TimeSyncServiceImpl.h>
#include <spot_comm/LeaseServiceImpl.h>
#include <spot_comm/LogAnnotationServiceImpl.h>
#include <spot_comm/RobotCommandServiceImpl.h>
//#include <spot_comm/RobotStateServiceImpl.h>
#include <spot_comm/PowerServiceImpl.h>
#include <ros/ros.h>
#include <ros/package.h>

using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::Status;
using grpc::ServerCredentials;

void read(const std::string& filename, std::string& data) {
  std::ifstream file(filename.c_str(), std::ios::in);
  if (file.is_open())
  {
    std::stringstream ss;
    ss << file.rdbuf();
    file.close();
    data = ss.str();
  }
  return;
}

void RunServer(ros::NodeHandle& n) {
  std::string server_address("localhost:50051");
  AuthServiceImpl authService;
  EstopServiceImpl estopService;
  DirectoryServiceImpl dirService;
  ImageServiceImpl imService;
  TimeSyncServiceImpl timeService;
  LeaseServiceImpl leaseService;
  LogAnnotationServiceImpl logService;
  RobotCommandServiceImpl commandService(n);
  //RobotStateServiceImpl stateService;
  PowerServiceImpl powerService;

  grpc::EnableDefaultHealthCheckService(true);
  grpc::reflection::InitProtoReflectionServerBuilderPlugin();
  ServerBuilder builder;

  std::shared_ptr<ServerCredentials> creds;
  std::string pathToPackage = ros::package::getPath("spot_comm");
  std::string key, cert, root;
  read(pathToPackage + "/include/certs/server.key", key);
  read(pathToPackage + "/include/certs/server.crt", cert);
  read(pathToPackage + "/include/certs/ca.crt", root);

  grpc::SslServerCredentialsOptions::PemKeyCertPair pkcp = {key, cert};
  grpc::SslServerCredentialsOptions ssl_opts;
  ssl_opts.pem_root_certs = root;
  ssl_opts.pem_key_cert_pairs.push_back(pkcp);
  creds = grpc::SslServerCredentials(ssl_opts);

  // Listen on the given address without any authentication mechanism.
  builder.AddListeningPort(server_address, creds);
  // Register "service" as the instance#include <spot_comm/Header.h> through which we'll communicate with
  // clients. In this case it corresponds to an *synchronous* service.
  
  builder.RegisterService(&authService); // change to dirService for directory test
  builder.RegisterService(&commandService); // change to dirService for directory test
  // Finally assemble the server.
  std::unique_ptr<Server> server(builder.BuildAndStart());
  std::cout << "Server listening on " << server_address << std::endl;

  //ros::spin();
  // Wait for the server to shutdown. Note that some other thread must be
  // responsible for shutting down the server for this call to ever return.
  server->Wait();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "spot_node");
  ros::NodeHandle n;
  grpc_init();
  RunServer(n);
  return 0;
}

