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

    Duration skew;
    skew = (prev_trip.server_rx() - prev_trip.client_tx()) + (prev_trip.server_tx() - prev_trip.client_rx());
    skew /= 2;

    prev->mutable_round_trip_time()->CopyFrom(trip_time);
    prev->mutable_clock_skew()->CopyFrom(skew);
}

void fillBestEst(TimeSyncEstimate* best, TimeSyncEstimate* prev) {
    best->mutable_round_trip_time()->CopyFrom(prev->round_trip_time());
    best->mutable_clock_skew()->CopyFrom(prev->clock_skew());
}

void fillHeader(ResponseHeader* header, const RequestHeader reqHeader) {
    header->mutable_request_received_timestamp()->CopyFrom(TimeUtil::GetCurrentTime());
    header->mutable_request_header()->CopyFrom(reqHeader);
}

Status TimeSyncServiceImpl::TimeSyncUpdate(ServerContext* context, const TimeSyncUpdateRequest* request, TimeSyncUpdateResponse* response) {
    // Information about previous round trip from request

    std::cout << "Repsonse start" << std::endl;

    // TimeSyncRoundTrip prev_trip = TimeSyncRoundTrip(request->previous_round_trip());
    // TimeSyncEstimate prev_est;
    // TimeSyncState state;

    std::cout<< "Repsonse start 2" << std::endl;

    // TimeSyncEstimate fields

    // Estimated time for one round trip
    // Duration total_time = (prev_trip.client_rx() - prev_trip.client_tx());
    // Duration server_processing = (prev_trip.server_tx() - prev_trip.server_rx());
    // Duration trip_time = total_time - server_processing;

    std::cout<< "Repsonse start 3" << std::endl;
    
    // Estimate of the difference between the client and server clocks
    // Duration skew;
    // skew = (prev_trip.server_rx() - prev_trip.client_tx()) + (prev_trip.server_tx() - prev_trip.client_rx());
    // skew /= 2;


    std::cout << "Repsonse start 4" << std::endl;
    

    // Fill in prev_est fields
    // prev_est.set_allocated_round_trip_time(&trip_time);
    // prev_est.set_allocated_clock_skew(&skew);

    fillPrevEst(response->mutable_previous_estimate(), request->previous_round_trip());

    std::cout << "Repsonse start 5" << std::endl;

    // TimeSyncState fields - for now, best estimate is prev one
    // Should be change best est to prev calculation if better than the curr best estimate
    // TimeSyncEstimate best_est;
    // TimeSyncEstimate fields
    // Duration* best_round = &trip_time;
    // best_round.set_seconds();
    // best_round.set_nanos();
    // Duration* best_skew = &skew;
    // best_skew.set_seconds();
    // best_skew.set_nanos();

    std::cout << "Repsonse start 6" << std::endl; 

    // best_est.set_allocated_round_trip_time(&trip_time);
    // best_est.set_allocated_clock_skew(&skew);

    fillBestEst(response->mutable_state()->mutable_best_estimate(), response->mutable_previous_estimate());

    std::cout << "Repsonse start 7" << std::endl;
    
    // Fill in state fields
    // state.set_allocated_best_estimate(&best_est);
    // state.set_status(TimeSyncState::STATUS_OK); // may need to change this
    // Timestamp curr_time = TimeUtil::GetCurrentTime();
    // state.set_allocated_measurement_time(&curr_time);

    fillState(response->mutable_state(), response->mutable_state()->mutable_best_estimate());

    std::cout << "Repsonse start 8" << std::endl;

    // Populate response message
    // ResponseHeader header = Header::generateResponseHeader(request->header());
    // response->mutable_header()->CopyFrom(Header::generateResponseHeader(request->header()));
    // response->set_allocated_header(&header);
    // response->set_allocated_previous_estimate(&prev_est);
    // response->set_allocated_state(&state);
    // std::string clock_identifier("spot-time-sync");
    // response->set_clock_identifier(clock_identifier);

    // response->mutable_state()->set_status(TimeSyncState::STATUS_OK);

    // response->mutable_header()->mutable_request_received_timestamp()->CopyFrom(TimeUtil::GetCurrentTime());
    // response->mutable_previous_estimate()->set_allocated_round_trip_time()
    // response->set_allocated_state(&state);
    // std::string clock_identifier("spot-time-sync");
    // response->set_clock_identifier(clock_identifier);

    fillHeader(response->mutable_header(), request->header());

    std::cout << "Response end" << std::endl;

    return Status::OK;
}