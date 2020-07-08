#!/usr/bin/env python3
import rospy

############################################
# Client interface for a Clearpath Jackal  #
# This interface utilizes the existing ROS #
# infastructure that s expected to be      #
# running on the jackal                    #
############################################


from abc import ABC, abstractmethod 
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from spot_comm.client import RobotClient

class JackalClient(RobotClient):

    def __init__(self, init_node=False):
        if init_node:
            rospy.init_node('jackal_ros_client_interface', anonymous=False)
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.scan_pub = rospy.Publisher('/scan', LaserScan, queue_size=10)

        self.cur_odom = Odometry()
        self.cur_scan = LaserScan()

        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)

    def odom_callback(self, data):
        self.cur_odom = data

    def scan_callback(self, data):
        self.cur_scan = data

    def get_odom(self):
        return self.cur_odom

    def get_vision(self):
        pass

    def get_laserscan(self):
        return self.cur_scan

    def get_robot_status(self):
        pass

    def set_waypoint(self):
        pass

    def set_estop_behavior(self):
        pass

    def start_robot(self):
        pass

    def stop_robot(self):
        pass

    def connect(self):
        pass


if __name__ == '__main__':
    print("Error! This should not be run directly")
