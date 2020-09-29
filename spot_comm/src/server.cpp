
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
#include <spot_comm/RobotStateServiceImpl.h>

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

void RunServer() {
  std::string server_address("localhost:50051");
  AuthServiceImpl authService;
  EstopServiceImpl estopService;
  DirectoryServiceImpl dirService;
  ImageServiceImpl imService;
  TimeSyncServiceImpl timeService;
  LeaseServiceImpl leaseService;
  LogAnnotationServiceImpl logService;
  RobotCommandServiceImpl commandService;
  RobotStateServiceImpl stateService;

  grpc::EnableDefaultHealthCheckService(true);
  grpc::reflection::InitProtoReflectionServerBuilderPlugin();
  ServerBuilder builder;

  /* std::shared_ptr<ServerCredentials> creds;

  std::string key, cert;
  read("key.pem", key);
  read("cert.pem", cert);

  grpc::SslServerCredentialsOptions::PemKeyCertPair pkcp ={key, cert};
  grpc::SslServerCredentialsOptions ssl_opts;
  ssl_opts.pem_root_certs="";
  ssl_opts.pem_key_cert_pairs.push_back(pkcp);
  creds = grpc::SslServerCredentials(ssl_opts); */

  // Listen on the given address without any authentication mechanism.
  builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
  // Register "service" as the instance#include <spot_comm/Header.h> through which we'll communicate with
  // clients. In this case it corresponds to an *synchronous* service.
  
  builder.RegisterService(&stateService); // change to dirService for directory test
  // Finally assemble the server.
  std::unique_ptr<Server> server(builder.BuildAndStart());
  std::cout << "Server listening on " << server_address << std::endl;

  // Wait for the server to shutdown. Note that some other thread must be
  // responsible for shutting down the server for this call to ever return.
  server->Wait();
}

int main(int argc, char** argv) {
  grpc_init();
  RunServer();
  return 0;
}

