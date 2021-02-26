#include <spot_comm/LogAnnotationServiceImpl.h>
#include <spot_comm/Header.h>

Status LogAnnotationServiceImpl::AddLogAnnotation(ServerContext* context, const AddLogAnnotationRequest* request, AddLogAnnotationResponse* response) {
  response->mutable_header()->CopyFrom(Header::generateResponseHeader(request->header()));
  for (int i = 0; i < request->annotations().text_messages_size(); i++) {
    std::cout << "Text Message: " << request->annotations().text_messages(i).message() << std::endl;
  }
  for (int i = 0; i < request->annotations().operator_messages_size(); i++) {
    std::cout << "Operator Message: " << request->annotations().operator_messages(i).message() << std::endl;
  }
  for (int i = 0; i < request->annotations().blob_data_size(); i++) {
    std::cout << "Blob Data: " << request->annotations().blob_data(i).data() << std::endl;
  }
  return Status::OK;
}
