// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: bosdyn/api/estop_service.proto

#include "bosdyn/api/estop_service.pb.h"

#include <algorithm>

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/wire_format_lite.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
namespace bosdyn {
namespace api {
}  // namespace api
}  // namespace bosdyn
static constexpr ::PROTOBUF_NAMESPACE_ID::Metadata* file_level_metadata_bosdyn_2fapi_2festop_5fservice_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_bosdyn_2fapi_2festop_5fservice_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_bosdyn_2fapi_2festop_5fservice_2eproto = nullptr;
const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_bosdyn_2fapi_2festop_5fservice_2eproto::offsets[1] = {};
static constexpr ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema* schemas = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::Message* const* file_default_instances = nullptr;

const char descriptor_table_protodef_bosdyn_2fapi_2festop_5fservice_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\036bosdyn/api/estop_service.proto\022\nbosdyn"
  ".api\032\026bosdyn/api/estop.proto2\354\004\n\014EstopSe"
  "rvice\022n\n\025RegisterEstopEndpoint\022(.bosdyn."
  "api.RegisterEstopEndpointRequest\032).bosdy"
  "n.api.RegisterEstopEndpointResponse\"\000\022t\n"
  "\027DeregisterEstopEndpoint\022*.bosdyn.api.De"
  "registerEstopEndpointRequest\032+.bosdyn.ap"
  "i.DeregisterEstopEndpointResponse\"\000\022S\n\014E"
  "stopCheckIn\022\037.bosdyn.api.EstopCheckInReq"
  "uest\032 .bosdyn.api.EstopCheckInResponse\"\000"
  "\022Y\n\016GetEstopConfig\022!.bosdyn.api.GetEstop"
  "ConfigRequest\032\".bosdyn.api.GetEstopConfi"
  "gResponse\"\000\022Y\n\016SetEstopConfig\022!.bosdyn.a"
  "pi.SetEstopConfigRequest\032\".bosdyn.api.Se"
  "tEstopConfigResponse\"\000\022k\n\024GetEstopSystem"
  "Status\022\'.bosdyn.api.GetEstopSystemStatus"
  "Request\032(.bosdyn.api.GetEstopSystemStatu"
  "sResponse\"\000B\023B\021EstopServiceProtob\006proto3"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_bosdyn_2fapi_2festop_5fservice_2eproto_deps[1] = {
  &::descriptor_table_bosdyn_2fapi_2festop_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_bosdyn_2fapi_2festop_5fservice_2eproto_sccs[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_bosdyn_2fapi_2festop_5fservice_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_bosdyn_2fapi_2festop_5fservice_2eproto = {
  false, false, descriptor_table_protodef_bosdyn_2fapi_2festop_5fservice_2eproto, "bosdyn/api/estop_service.proto", 720,
  &descriptor_table_bosdyn_2fapi_2festop_5fservice_2eproto_once, descriptor_table_bosdyn_2fapi_2festop_5fservice_2eproto_sccs, descriptor_table_bosdyn_2fapi_2festop_5fservice_2eproto_deps, 0, 1,
  schemas, file_default_instances, TableStruct_bosdyn_2fapi_2festop_5fservice_2eproto::offsets,
  file_level_metadata_bosdyn_2fapi_2festop_5fservice_2eproto, 0, file_level_enum_descriptors_bosdyn_2fapi_2festop_5fservice_2eproto, file_level_service_descriptors_bosdyn_2fapi_2festop_5fservice_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_bosdyn_2fapi_2festop_5fservice_2eproto = (static_cast<void>(::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_bosdyn_2fapi_2festop_5fservice_2eproto)), true);
namespace bosdyn {
namespace api {

// @@protoc_insertion_point(namespace_scope)
}  // namespace api
}  // namespace bosdyn
PROTOBUF_NAMESPACE_OPEN
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>