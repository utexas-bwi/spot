#include <spot_comm/RobotCommandServiceImpl.h>
#include <spot_comm/Header.h>

Status RobotCommandServiceImpl::RobotCommand(ServerContext* context, const RobotCommandRequest* request, RobotCommandResponse* response) {
    return Status::OK;
}

Status RobotCommandServiceImpl::RobotCommandFeedback(ServerContext* context, const RobotCommandFeedbackRequest* request, RobotCommandFeedbackResponse* response) {
    return Status::OK;
}

Status RobotCommandServiceImpl::ClearBehaviorFault(ServerContext* context, const ClearBehaviorFaultRequest* request, ClearBehaviorFaultResponse* response) {
    return Status::OK;
}