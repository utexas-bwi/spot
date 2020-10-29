#include <spot_comm/TimeSyncServiceImpl.h>
#include <spot_comm/Header.h>

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

void fillHeader(ResponseHeader* header, const RequestHeader reqHeader) {
  header->mutable_request_received_timestamp()->CopyFrom(TimeUtil::GetCurrentTime());
  header->mutable_request_header()->CopyFrom(reqHeader);
}

Status TimeSyncServiceImpl::TimeSyncUpdate(ServerContext* context, const TimeSyncUpdateRequest* request, TimeSyncUpdateResponse* response) {
  // Mark when request was received and copy over request header  
  fillHeader(response->mutable_header(), request->header());
  
  // Calculate round trip time and clock skew from previous exchange
  fillPrevEst(response->mutable_previous_estimate(), request->previous_round_trip());

  // Populate rest of response fields -- clock identifier and document when response was sent
  response->set_clock_identifier(request->clock_identifier());
  response->mutable_state()->set_status(TimeSyncState::STATUS_OK);
  response->mutable_header()->mutable_response_timestamp()->CopyFrom(TimeUtil::GetCurrentTime());

  return Status::OK;
}
