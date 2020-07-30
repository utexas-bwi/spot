#ifndef HEADER_H
#define HEADER_H

#include "bosdyn/api/header.grpc.pb.h"

using bosdyn::api::ResponseHeader;
using bosdyn::api::RequestHeader;
using google::protobuf::Message;

class Header {
public:
  ResponseHeader generateResponseHeader(Message message);
};

#endif
