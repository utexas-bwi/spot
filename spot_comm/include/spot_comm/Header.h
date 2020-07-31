#ifndef HEADER_H
#define HEADER_H

#include "bosdyn/api/header.grpc.pb.h"
#include <google/protobuf/util/time_util.h>

using bosdyn::api::ResponseHeader;
using bosdyn::api::RequestHeader;
using google::protobuf::Timestamp;
using google::protobuf::util::TimeUtil;

class Header {
public:
  static ResponseHeader generateResponseHeader(RequestHeader request_header) {
    ResponseHeader response;
    Timestamp request_received_timestamp = TimeUtil::GetCurrentTime();
    response.set_allocated_request_received_timestamp(&request_received_timestamp);
    RequestHeader request_header_copy = RequestHeader(request_header);
    response.set_allocated_request_header(&request_header_copy);
    Timestamp response_timestamp = TimeUtil::GetCurrentTime();
    response.set_allocated_response_timestamp(&response_timestamp);
    return response;
  }
};

#endif
