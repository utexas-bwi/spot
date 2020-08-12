#include <spot_comm/TimeSyncServiceImpl.h>
#include <spot_comm/Header.h>

void fillState(TimeSyncState* state, TimeSyncEstimate* best) {
    state->mutable_best_estimate()->CopyFrom(*best);
    state->set_status(TimeSyncState::STATUS_OK); // may need to change this
    state->mutable_measurement_time()->CopyFrom(TimeUtil::GetCurrentTime());
}

void fillPrevEst(TimeSyncEstimate* prev, const TimeSyncRoundTrip prev_trip) {
    Duration total_time = (prev_trip.client_rx() - prev_trip.client_tx());
    Duration server_processing = (prev_trip.server_tx() - prev_trip.server_rx());
    Duration trip_time = total_time - server_processing;

    std::cout << "s: " << trip_time.seconds() << " ns: " << trip_time.nanos() << std::endl;

    Duration skew;
    skew = (prev_trip.server_rx() - prev_trip.client_tx()) + (prev_trip.server_tx() - prev_trip.client_rx());
    skew /= 2;

    std::cout << "skew: " << skew.seconds() << std::endl;

    prev->mutable_round_trip_time()->CopyFrom(trip_time);
    prev->mutable_clock_skew()->CopyFrom(skew);
}

void fillBestEst(TimeSyncEstimate* best, TimeSyncEstimate* prev) {
    best->mutable_round_trip_time()->CopyFrom(prev->round_trip_time());
    best->mutable_clock_skew()->CopyFrom(prev->clock_skew());
}

void fillHeader(ResponseHeader* header, const RequestHeader reqHeader) {
    header->mutable_request_received_timestamp()->CopyFrom(TimeUtil::GetCurrentTime());
    // header->mutable_request_received_timestamp()->set_seconds(header->request_received_timestamp().seconds());
    header->mutable_request_header()->CopyFrom(reqHeader);
}

Status TimeSyncServiceImpl::TimeSyncUpdate(ServerContext* context, const TimeSyncUpdateRequest* request, TimeSyncUpdateResponse* response) {
    // Mark when request was received and copy over request header    
    fillHeader(response->mutable_header(), request->header());
    
    // std::cout << "Repsonse start" << std::endl;

    // Calculate round trip time and clock skew from previous exchange
    fillPrevEst(response->mutable_previous_estimate(), request->previous_round_trip());

    // std::cout << "Repsonse start 5" << std::endl;

    // Update best estimate if applicable
    fillBestEst(response->mutable_state()->mutable_best_estimate(), response->mutable_previous_estimate());

    // std::cout << "Repsonse start 7" << std::endl;

    // Update state fields
    fillState(response->mutable_state(), response->mutable_state()->mutable_best_estimate());

    // std::cout << "Repsonse start 8" << std::endl;

    // Populate rest of response fields -- clock idenfier and document when response was sent
    response->set_clock_identifier(request->clock_identifier());
    response->mutable_header()->mutable_response_timestamp()->CopyFrom(TimeUtil::GetCurrentTime());
    // response->mutable_header()->mutable_response_timestamp()->set_seconds(response->header().response_timestamp().seconds());

    // std::cout << "Response end" << std::endl;

    return Status::OK;
}