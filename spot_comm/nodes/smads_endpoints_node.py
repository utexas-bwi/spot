#!/usr/bin/env python3

import rospy
import threading

from enum import Enum

from spot_comm.client import JackalClient
from spot_comm.client import SpotClient
from spot_comm.client import RobotClient

from spot_comm.interface import RobotSensorInterface
from spot_comm.interface import RobotNavigationInterface

class RobotType:
    SPOT    = 1
    JACKAL  = 2

    platform_map = {
        SPOT : SpotClient(),
        JACKAL : JackalClient(),
    }

class SMADSROS:
    def __init__(self, client, sensor_poll_rate, robot_prefix="smads_platform"):
        self.client = client
        self.robot_prefix = robot_prefix
        self.client_mutex = threading.Lock()
        self.sensor_interface = RobotSensorInterface(client, self.client_mutex, sensor_poll_rate, robot_prefix)
        self.navigation_interface = RobotNavigationInterface(client, self.client_mutex, robot_prefix)

    def start(self):
        x = threading.Thread(target=self.sensor_interface.start)
        y = threading.Thread(target=self.navigation_interface.start)
        #self.sensor_interface.start()
        #self.navigation_interface.start()
        x.start()
        y.start()
        rospy.spin()

if __name__ == '__main__':
    try:
        rospy.init_node('smads_ros_node', anonymous=False)
        platform = RobotType.JACKAL
        client = RobotType.platform_map[platform]

        smadsros = SMADSROS(client, 10, "smads_platform")
        smadsros.start()

    except rospy.ROSInterruptException:
        pass

