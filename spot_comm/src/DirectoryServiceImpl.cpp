#include <spot_comm/DirectoryServiceImpl.h>
#include <spot_comm/Header.h>

Status DirectoryServiceImpl::GetServiceEntry(ServerContext* context, const GetServiceEntryRequest* request,
                GetServiceEntryResponse* response) {
  std::cout << "Service Name: " << request->service_name() << std::endl;
  ServiceEntry* entry;
  Timestamp timestamp = TimeUtil::GetCurrentTime();
  entry->set_name("testService");
  entry->set_type("testType");
  entry->set_authority("testAuthority"); 
  entry->set_allocated_last_update(&timestamp);
  entry->set_user_token_required(false);
  entry->set_permission_required("");
  response->set_status(GetServiceEntryResponse::STATUS_OK);
  response->set_allocated_service_entry(entry);
  return Status::OK;
}

Status DirectoryServiceImpl::ListServiceEntries(ServerContext* context, const ListServiceEntriesRequest* request,
                ListServiceEntriesResponse* response) {
  return Status::OK;
}
