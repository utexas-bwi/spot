#!/bin/bash

pushd spot_comm

protoc -I . --grpc_out=./include --plugin=protoc-gen-grpc=`which grpc_cpp_plugin` bosdyn/api/*.proto bosdyn/api/spot/*.proto bosdyn/api/spot_cam/*.proto bosdyn/api/mission/*.proto bosdyn/api/graph_nav/*.proto

protoc -I . --cpp_out=./include bosdyn/api/*.proto bosdyn/api/spot/*.proto bosdyn/api/spot_cam/*.proto bosdyn/api/mission/*.proto bosdyn/api/graph_nav/*.proto

popd

