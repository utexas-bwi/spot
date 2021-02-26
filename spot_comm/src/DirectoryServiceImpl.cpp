#include <spot_comm/DirectoryServiceImpl.h>
#include <spot_comm/Header.h>

#define SERVICE_COUNT 12
std::string serviceNames[SERVICE_COUNT] = {"auth", "estop", "directory", "image", "time-sync", "lease", "license", "log-annotation", "power", "robot-command", "robot-state", "robot-id"};
std::string serviceTypes[SERVICE_COUNT] = {"AuthService", "EstopService", "DirectoryService", "ImageService", "TimeSyncService", "LeaseService", "LicenseService", "LogAnnotationService", "PowerService", "RobotCommandService", "RobotStateService", "RobotIdService"};

ServiceEntry fillServiceEntry(std::string serviceName, std::string serviceType) {
  ServiceEntry entry;
  entry.set_name(serviceName);
  entry.set_type("bosdyn.api." + serviceType);
  entry.set_authority(serviceName + ".spot.robot"); 
  entry.mutable_last_update()->CopyFrom(TimeUtil::GetCurrentTime());
  entry.set_user_token_required(false);
  return entry;
}

Status DirectoryServiceImpl::GetServiceEntry(ServerContext* context, const GetServiceEntryRequest* request, GetServiceEntryResponse* response) {
  response->mutable_header()->CopyFrom(Header::generateResponseHeader(request->header()));  
  std::cout << "Service Name: " << request->service_name() << std::endl;
  
  for (int i = 0; i < SERVICE_COUNT; i++) {
    if (serviceNames[i] == request->service_name()) {
      response->set_status(GetServiceEntryResponse::STATUS_OK);
      response->mutable_service_entry()->CopyFrom(fillServiceEntry(serviceNames[i], serviceTypes[i]));
      return Status::OK;
    }
  }
  
  response->set_status(GetServiceEntryResponse::STATUS_NONEXISTENT_SERVICE);
  return Status::OK;
}

Status DirectoryServiceImpl::ListServiceEntries(ServerContext* context, const ListServiceEntriesRequest* request, ListServiceEntriesResponse* response) {
  response->mutable_header()->CopyFrom(Header::generateResponseHeader(request->header()));
  for (int i = 0; i < SERVICE_COUNT; i++) {
    response->add_service_entries()->CopyFrom(fillServiceEntry(serviceNames[i], serviceTypes[i]));
  }
  return Status::OK;
}
