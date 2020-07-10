#!/usr/bin/env python3

import rospy
import sensor_msgs

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from spot_comm.client import RobotClient

class RobotSensorInterface:

    def __init__(self, client, mutex, poll_rate=10, robot_prefix="smads_platform"):
        self.client = client
        self.mutex = mutex
        self.poll_rate = poll_rate
        self.robot_prefix = robot_prefix

        self.odom_pub = None
        self.scan_pub = None
        odom_postfix = rospy.get_param("smads_output_odom_topic")
        scan_postfix = rospy.get_param("smads_output_scan_topic")
        self.odom_pub = rospy.Publisher(self.robot_prefix + odom_postfix, Odometry, queue_size=10)
        self.scan_pub = rospy.Publisher(self.robot_prefix + scan_postfix, LaserScan, queue_size=10)
    
    def poll(self):
        r = rospy.Rate(self.poll_rate)
        while not rospy.is_shutdown():
            scan_data = LaserScan()
            odom_data = Odometry()
            with self.mutex:
                scan_data = self.client.get_laserscan()
                odom_data = self.client.get_odom()

            self.scan_pub.publish(scan_data)
            self.odom_pub.publish(odom_data)
            
            r.sleep()

    def start(self):
        rospy.loginfo("robot_sensor_interface active")
        self.poll()
        rospy.loginfo("Polling of Client sensor data stopped.")

