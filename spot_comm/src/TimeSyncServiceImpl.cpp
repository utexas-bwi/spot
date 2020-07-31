#include <spot_comm/TimeSyncServiceImpl.h>
#include <spot_comm/Header.h>

Status TimeSyncServiceImpl::TimeSyncUpdate(ServerContext* context, const TimeSyncUpdateRequest* request, TimeSyncUpdateResponse* response) {
    // Information about previous round trip from request
    TimeSyncRoundTrip prev_trip = TimeSyncRoundTrip(request->previous_round_trip());
    TimeSyncEstimate* prev_est;
    TimeSyncState* state;

    // TimeSyncEstimate fields

    // Estimated time for one request-response exchange
    Duration* trip_time;
    // Client time when the message was sent in secs
    int64_t c_tx = TimeUtil::TimestampToSeconds(prev_trip.client_tx());
    // Server time when the message was received in secs
    int64_t s_rx = TimeUtil::TimestampToSeconds(prev_trip.server_rx());
    // Server time when the response was sent in secs
    int64_t s_tx = TimeUtil::TimestampToSeconds(prev_trip.server_tx());
    // Client time when the response was received in secs
    int64_t c_rx = TimeUtil::TimestampToSeconds(prev_trip.client_rx());
    int64_t round_trip_secs = (s_rx - c_tx) + (c_rx - s_tx);
    trip_time->set_seconds(round_trip_secs);
    trip_time->set_nanos(round_trip_secs * 1000000000);
    
    // Estimate of the difference between the client and server clocks
    Duration* skew;
    // skew.set_seconds();
    // skew.set_nanos();

    // Fill in prev_est fields
    prev_est->set_allocated_round_trip_time(trip_time);
    prev_est->set_allocated_clock_skew(skew);

    // TimeSyncState fields - for now, best estimate is prev one
    TimeSyncEstimate* best_est;
    // TimeSyncEstimate fields
    Duration* best_round = trip_time;
    // best_round.set_seconds();
    // best_round.set_nanos();
    Duration* best_skew = skew;
    // best_skew.set_seconds();
    // best_skew.set_nanos();
    // Timestamp best_est_time;
    // best_est_time.set_seconds();
    // best_est_time.set_nanos();
    best_est->set_allocated_round_trip_time(best_round);
    best_est->set_allocated_clock_skew(best_skew);
    
    // Fill in state fields
    state->set_allocated_best_estimate(best_est);
    state->set_status(TimeSyncState::STATUS_OK); // may need to change this
    Timestamp curr_time = TimeUtil::GetCurrentTime();
    state->set_allocated_measurement_time(&curr_time);

    // Populate response message
    ResponseHeader header = Header::generateResponseHeader(request->header());
    response->set_allocated_header(&header);
    response->set_allocated_previous_estimate(prev_est);
    response->set_allocated_state(state);
    response->set_clock_identifier(request->clock_identifier());

    return Status::OK;
}