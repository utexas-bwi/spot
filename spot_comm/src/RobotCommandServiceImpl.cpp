#include <spot_comm/RobotCommandServiceImpl.h>
#include <spot_comm/Header.h>

Status RobotCommandServiceImpl::RobotCommand(ServerContext* context, const RobotCommandRequest* request, RobotCommandResponse* response) {
    // header
    response->mutable_header()->CopyFrom(Header::generateResponseHeader(reqeust->header()));
    
    // lease_use_result
    response->mutable_lease_use_result()->set_status(LeaseUseResult::STATUS_OK);
    response->mutable_lease_use_result()->mutable_owner()->set_client_name("testClient");
    response->mutable_lease_use_result()->mutable_owner()->set_user_name("testUser");
    response->mutable_lease_use_result()->mutable_attempted_lease()->CopyFrom(request->lease());
    response->mutable_lease_use_result()->mutable_previous_lease()->CopyFrom(request->lease()); // may need to change

    // status
    response->set_status(RobotCommandResponse::STATUS_OK);

    // id
    response->set_robot_command_id(100);

    // message
    response->set_message("Id: " + response->robot_command_id());
    
    return Status::OK;
}

Status RobotCommandServiceImpl::RobotCommandFeedback(ServerContext* context, const RobotCommandFeedbackRequest* request, RobotCommandFeedbackResponse* response) {
    // header
    response->mutable_header()->CopyFrom(Header::generateResponseHeader(request->header()));

    // status
    response->set_status(RobotCommandFeedbackResponse::STATUS_OK);

    // feedback
    response->mutable_feedback()->mutable_full_body_feedback()->mutable_safe_power_off_feedback()->set_status(SafePowerOffCommandFeedback::STATUS_POWERED_OFF);
    response->mutable_feedback()->mutable_mobility_feedback()->mutable_se2_trajectory_feedback()->set_status(SE2TrajectoryCommandFeedback::STATUS_AT_GOAL);
    response->mutable_feedback()->mutable_mobility_feedback()->mutable_sit_feedback()->set_status(SitCommandFeedback::STATUS_IS_SITTING);
    response->mutable_feedback()->mutable_mobility_feedback()->mutable_stand_feedback()->set_status(StandCommandFeedback::STATUS_IS_STANDING);
    

    // message
    response->set_message("Trajectory feedback status: " + response->feedback().mobility_feedback().se2_trajectory_feedback().status());
    
    return Status::OK;
}

Status RobotCommandServiceImpl::ClearBehaviorFault(ServerContext* context, const ClearBehaviorFaultRequest* request, ClearBehaviorFaultResponse* response) {
    return Status::OK;
}