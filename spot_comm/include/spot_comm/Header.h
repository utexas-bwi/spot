#ifndef HEADER_H
#define HEADER_H

#include "bosdyn/api/header.grpc.pb.h"
#include <google/protobuf/util/time_util.h>

using bosdyn::api::ResponseHeader;
using bosdyn::api::RequestHeader;
using bosdyn::api::CommonError;
using google::protobuf::Timestamp;
using google::protobuf::util::TimeUtil;

class Header {
public:
  static ResponseHeader generateResponseHeader(RequestHeader request_header) {
    ResponseHeader response;
    response.mutable_request_received_timestamp()->CopyFrom(TimeUtil::GetCurrentTime());
    response.mutable_request_header()->CopyFrom(request_header);
    response.mutable_response_timestamp()->CopyFrom(TimeUtil::GetCurrentTime());
    CommonError error;
    error.set_code(CommonError::CODE_OK);
    error.set_message("OK");
    response.mutable_error()->CopyFrom(error);
    return response;
  }
};

#endif
