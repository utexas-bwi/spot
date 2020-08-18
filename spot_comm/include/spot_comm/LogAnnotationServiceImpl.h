#ifndef LOG_ANNOTATION_SERVICE_IMPL_H
#define LOG_ANNOTATION_SERVICE_IMPL_H

#include "bosdyn/api/log_annotation_service.grpc.pb.h"

using bosdyn::api::AddLogAnnotationRequest;
using bosdyn::api::AddLogAnnotationResponse;
using bosdyn::api::LogAnnotationService;
using grpc::Status;
using grpc::ServerContext;

class LogAnnotationServiceImpl final : public LogAnnotationService::Service {
public:
  Status AddLogAnnotation(ServerContext* context, const AddLogAnnotationRequest* request, AddLogAnnotationResponse* response) override;
};

#endif
