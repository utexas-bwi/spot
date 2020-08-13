#include <spot_comm/DirectoryServiceImpl.h>
#include <spot_comm/Header.h>

void fillServiceEntry(ServiceEntry* entry) {
  entry->set_name("testService");
  entry->set_type("testType");
  entry->set_authority("testAuthority"); 
  entry->mutable_last_update()->CopyFrom(TimeUtil::GetCurrentTime());
  entry->set_user_token_required(false);
  entry->set_permission_required("");
}

Status DirectoryServiceImpl::GetServiceEntry(ServerContext* context, const GetServiceEntryRequest* request,
                GetServiceEntryResponse* response) {
  std::cout << "Service Name: " << request->service_name() << std::endl;
  response->set_status(GetServiceEntryResponse::STATUS_OK);
  if (response->status() == GetServiceEntryResponse::STATUS_OK)
    fillServiceEntry(response->mutable_service_entry());
  return Status::OK;
}

Status DirectoryServiceImpl::ListServiceEntries(ServerContext* context, const ListServiceEntriesRequest* request,
                ListServiceEntriesResponse* response) {
  ServiceEntry entry;
  fillServiceEntry(&entry);
  response -> add_service_entries()->CopyFrom(entry);
  return Status::OK;
}
