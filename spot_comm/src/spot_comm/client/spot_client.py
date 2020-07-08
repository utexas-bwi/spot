#!/usr/bin/env python3

import bosdyn.client.util

from abc import ABC, abstractmethod 
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.robot_command import RobotCommandClient, RobotCommandBuilder, blocking_stand
from bosdyn.api import estop_pb2
from bosdyn.api import geometry_pb2 as geo
from bosdyn.client import math_helpers
from bosdyn.client.estop import EstopClient, EstopEndpoint, EstopKeepAlive
from bosdyn.client.frame_helpers import ODOM_FRAME_NAME, VISION_FRAME_NAME, get_odom_tform_body, get_vision_tform_body
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive 
from spot_comm.client import RobotClient


class SpotClient(RobotClient):

    def __init__(self):
        pass

    def connect(self, app_token, hostname, username, password, initialize_motion=True):
         # Create robot object.
        sdk = bosdyn.client.create_standard_sdk('RobotCommandMaster')
        sdk.load_app_token(app_token)
        robot = sdk.create_robot(hostname)
        robot.authenticate(username, password)

        # Check that an estop is connected with the robot so that the robot commands can be executed.
        verify_estop(robot)

        # Create the lease client.
        lease_client = robot.ensure_client(LeaseClient.default_service_name)
        lease = lease_client.acquire()
        robot.time_sync.wait_for_sync()
        lk = bosdyn.client.lease.LeaseKeepAlive(lease_client)

        # Setup clients for the robot state and robot command services.
        robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)
        robot_command_client = robot.ensure_client(RobotCommandClient.default_service_name)

        # Power on the robot and stand it up.
        robot.power_on()
        if initialize_motion:
            blocking_stand(robot_command_client)



if __name__ == '__main__':
    print("Error! This should not be run directly")
