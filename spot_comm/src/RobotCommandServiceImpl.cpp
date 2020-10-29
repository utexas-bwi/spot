#include <spot_comm/RobotCommandServiceImpl.h>
#include <spot_comm/Header.h>

extern PowerState_MotorPowerState currentPowerState;

double getXVel(const RobotCommandRequest* request) {
  return request->command().mobility_command().se2_velocity_request().velocity().linear().x();
}

double getYVel(const RobotCommandRequest* request) {
  return request->command().mobility_command().se2_velocity_request().velocity().linear().y();
}

double getAngularVel(const RobotCommandRequest* request) {
  return request->command().mobility_command().se2_velocity_request().velocity().angular();
}

RobotCommandServiceImpl::RobotCommandServiceImpl(ros::NodeHandle &n): nh(n), vel(nh) {}

Status RobotCommandServiceImpl::RobotCommand(ServerContext* context, const RobotCommandRequest* request, RobotCommandResponse* response) {
  // header
  response->mutable_header()->CopyFrom(Header::generateResponseHeader(request->header()));
  
  // lease_use_result
  response->mutable_lease_use_result()->set_status(LeaseUseResult::STATUS_OK);
  response->mutable_lease_use_result()->mutable_owner()->set_client_name("testClient");
  response->mutable_lease_use_result()->mutable_owner()->set_user_name("testUser");
  response->mutable_lease_use_result()->mutable_attempted_lease()->CopyFrom(request->lease());
  response->mutable_lease_use_result()->mutable_previous_lease()->CopyFrom(request->lease()); // may need to change

  // status
  response->set_status(RobotCommandResponse::STATUS_OK);

  // id
  response->set_robot_command_id(10);

  // message
  response->set_message("RobotCommandRequest received");

  // Publish twist message
  if (request->command().has_mobility_command() && request->command().mobility_command().has_se2_velocity_request()) {
    vel.executeCommand(getXVel(request), getYVel(request), getAngularVel(request));
  } else if (request->command().has_full_body_command() && request->command().full_body_command().has_safe_power_off_request()) {
    currentPowerState = PowerState::STATE_OFF;
  }
    
  return Status::OK;
}

Status RobotCommandServiceImpl::RobotCommandFeedback(ServerContext* context, const RobotCommandFeedbackRequest* request, RobotCommandFeedbackResponse* response) {
  // header
  response->mutable_header()->CopyFrom(Header::generateResponseHeader(request->header()));

  // status
  response->set_status(RobotCommandFeedbackResponse::STATUS_PROCESSING);

  // feedback
  response->mutable_feedback()->mutable_full_body_feedback()->mutable_safe_power_off_feedback()->set_status(SafePowerOffCommand_Feedback::STATUS_POWERED_OFF);
  response->mutable_feedback()->mutable_mobility_feedback()->mutable_se2_trajectory_feedback()->set_status(SE2TrajectoryCommand_Feedback::STATUS_AT_GOAL);
  response->mutable_feedback()->mutable_mobility_feedback()->mutable_sit_feedback()->set_status(SitCommand_Feedback::STATUS_IS_SITTING);
  response->mutable_feedback()->mutable_mobility_feedback()->mutable_stand_feedback()->set_status(StandCommand_Feedback::STATUS_IS_STANDING);
   
  // message
  response->set_message("Trajectory feedback status: " + response->feedback().mobility_feedback().se2_trajectory_feedback().status());
 
  return Status::OK;
}

Status RobotCommandServiceImpl::ClearBehaviorFault(ServerContext* context, const ClearBehaviorFaultRequest* request, ClearBehaviorFaultResponse* response) {
    return Status::OK;
}
