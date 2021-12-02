#include <spot_comm/LocalGridServiceImpl.h>
#include <spot_comm/Header.h>

Status LocalGridServiceImpl::GetLocalGridTypes(ServerContext* context, const GetLocalGridTypesRequest* request, GetLocalGridTypesResponse* response) {
  response->mutable_header()->CopyFrom(Header::generateResponseHeader(request->header()));
  return Status::OK;
}

Status LocalGridServiceImpl::GetLocalGrids(ServerContext* context, const GetLocalGridsRequest* request, GetLocalGridsResponse* response) {
  response->mutable_header()->CopyFrom(Header::generateResponseHeader(request->header()));
  response->set_num_local_grid_errors(request->local_grid_requests_size());
  return Status::OK;
}
