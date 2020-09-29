// Generated by the gRPC C++ plugin.
// If you make any local change, they will be lost.
// source: bosdyn/api/payload_service.proto

#include "bosdyn/api/payload_service.pb.h"
#include "bosdyn/api/payload_service.grpc.pb.h"

#include <functional>
#include <grpcpp/impl/codegen/async_stream.h>
#include <grpcpp/impl/codegen/async_unary_call.h>
#include <grpcpp/impl/codegen/channel_interface.h>
#include <grpcpp/impl/codegen/client_unary_call.h>
#include <grpcpp/impl/codegen/client_callback.h>
#include <grpcpp/impl/codegen/message_allocator.h>
#include <grpcpp/impl/codegen/method_handler.h>
#include <grpcpp/impl/codegen/rpc_service_method.h>
#include <grpcpp/impl/codegen/server_callback.h>
#include <grpcpp/impl/codegen/server_callback_handlers.h>
#include <grpcpp/impl/codegen/server_context.h>
#include <grpcpp/impl/codegen/service_type.h>
#include <grpcpp/impl/codegen/sync_stream.h>
namespace bosdyn {
namespace api {

static const char* PayloadService_method_names[] = {
  "/bosdyn.api.PayloadService/ListPayloads",
};

std::unique_ptr< PayloadService::Stub> PayloadService::NewStub(const std::shared_ptr< ::grpc::ChannelInterface>& channel, const ::grpc::StubOptions& options) {
  (void)options;
  std::unique_ptr< PayloadService::Stub> stub(new PayloadService::Stub(channel));
  return stub;
}

PayloadService::Stub::Stub(const std::shared_ptr< ::grpc::ChannelInterface>& channel)
  : channel_(channel), rpcmethod_ListPayloads_(PayloadService_method_names[0], ::grpc::internal::RpcMethod::NORMAL_RPC, channel)
  {}

::grpc::Status PayloadService::Stub::ListPayloads(::grpc::ClientContext* context, const ::bosdyn::api::ListPayloadsRequest& request, ::bosdyn::api::ListPayloadsResponse* response) {
  return ::grpc::internal::BlockingUnaryCall(channel_.get(), rpcmethod_ListPayloads_, context, request, response);
}

void PayloadService::Stub::experimental_async::ListPayloads(::grpc::ClientContext* context, const ::bosdyn::api::ListPayloadsRequest* request, ::bosdyn::api::ListPayloadsResponse* response, std::function<void(::grpc::Status)> f) {
  ::grpc::internal::CallbackUnaryCall(stub_->channel_.get(), stub_->rpcmethod_ListPayloads_, context, request, response, std::move(f));
}

void PayloadService::Stub::experimental_async::ListPayloads(::grpc::ClientContext* context, const ::grpc::ByteBuffer* request, ::bosdyn::api::ListPayloadsResponse* response, std::function<void(::grpc::Status)> f) {
  ::grpc::internal::CallbackUnaryCall(stub_->channel_.get(), stub_->rpcmethod_ListPayloads_, context, request, response, std::move(f));
}

void PayloadService::Stub::experimental_async::ListPayloads(::grpc::ClientContext* context, const ::bosdyn::api::ListPayloadsRequest* request, ::bosdyn::api::ListPayloadsResponse* response, ::grpc::experimental::ClientUnaryReactor* reactor) {
  ::grpc::internal::ClientCallbackUnaryFactory::Create(stub_->channel_.get(), stub_->rpcmethod_ListPayloads_, context, request, response, reactor);
}

void PayloadService::Stub::experimental_async::ListPayloads(::grpc::ClientContext* context, const ::grpc::ByteBuffer* request, ::bosdyn::api::ListPayloadsResponse* response, ::grpc::experimental::ClientUnaryReactor* reactor) {
  ::grpc::internal::ClientCallbackUnaryFactory::Create(stub_->channel_.get(), stub_->rpcmethod_ListPayloads_, context, request, response, reactor);
}

::grpc::ClientAsyncResponseReader< ::bosdyn::api::ListPayloadsResponse>* PayloadService::Stub::AsyncListPayloadsRaw(::grpc::ClientContext* context, const ::bosdyn::api::ListPayloadsRequest& request, ::grpc::CompletionQueue* cq) {
  return ::grpc::internal::ClientAsyncResponseReaderFactory< ::bosdyn::api::ListPayloadsResponse>::Create(channel_.get(), cq, rpcmethod_ListPayloads_, context, request, true);
}

::grpc::ClientAsyncResponseReader< ::bosdyn::api::ListPayloadsResponse>* PayloadService::Stub::PrepareAsyncListPayloadsRaw(::grpc::ClientContext* context, const ::bosdyn::api::ListPayloadsRequest& request, ::grpc::CompletionQueue* cq) {
  return ::grpc::internal::ClientAsyncResponseReaderFactory< ::bosdyn::api::ListPayloadsResponse>::Create(channel_.get(), cq, rpcmethod_ListPayloads_, context, request, false);
}

PayloadService::Service::Service() {
  AddMethod(new ::grpc::internal::RpcServiceMethod(
      PayloadService_method_names[0],
      ::grpc::internal::RpcMethod::NORMAL_RPC,
      new ::grpc::internal::RpcMethodHandler< PayloadService::Service, ::bosdyn::api::ListPayloadsRequest, ::bosdyn::api::ListPayloadsResponse>(
          [](PayloadService::Service* service,
             ::grpc::ServerContext* ctx,
             const ::bosdyn::api::ListPayloadsRequest* req,
             ::bosdyn::api::ListPayloadsResponse* resp) {
               return service->ListPayloads(ctx, req, resp);
             }, this)));
}

PayloadService::Service::~Service() {
}

::grpc::Status PayloadService::Service::ListPayloads(::grpc::ServerContext* context, const ::bosdyn::api::ListPayloadsRequest* request, ::bosdyn::api::ListPayloadsResponse* response) {
  (void) context;
  (void) request;
  (void) response;
  return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
}


}  // namespace bosdyn
}  // namespace api
