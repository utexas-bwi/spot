#include <spot_comm/RobotIdServiceImpl.h>
#include <spot_comm/Header.h>

Status RobotIdServiceImpl::GetRobotId(ServerContext* context, const RobotIdRequest* request, RobotIdResponse* response) {
  response->mutable_header()->CopyFrom(Header::generateResponseHeader(request->header()));
  RobotId robotId;
  std::string serialNumber("testSerialNumber");
  std::string species("spot");
  std::string versionString("2.0.2");
  RobotSoftwareRelease softwareRelease;
  SoftwareVersion version;
  version.set_major_version(2);
  version.set_minor_version(0);
  version.set_patch_level(2);
  std::string name("testName");
  std::string type("testType");
  std::string changeset("testChangeset");
  std::string apiVersion("2.14.5");
  std::string buildInformation("testBuildInformation");
  softwareRelease.mutable_version()->CopyFrom(version);
  softwareRelease.set_name(name);  
  softwareRelease.set_type(type);
  softwareRelease.mutable_changeset_date()->CopyFrom(TimeUtil::NanosecondsToTimestamp(1556697600000000000));
  softwareRelease.set_changeset(changeset);
  softwareRelease.set_api_version(apiVersion);
  softwareRelease.set_build_information(buildInformation);
  softwareRelease.mutable_install_date()->CopyFrom(TimeUtil::NanosecondsToTimestamp(1556697600000000000));
  std::string computerSerialNumber("testComputerSerialNumber");
  robotId.set_serial_number(serialNumber);
  robotId.set_species(species);
  robotId.set_version(versionString);
  robotId.mutable_software_release()->CopyFrom(softwareRelease);
  robotId.set_computer_serial_number(computerSerialNumber);
  response->mutable_robot_id()->CopyFrom(robotId);
  return Status::OK;
}
