#!/usr/bin/env python3

import rospy
import sensor_msgs

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from spot_comm.client import SpotClient

def initialize_node():
    rospy.init_node('spot_sensor_node', anonymous=False)
    rospy.loginfo("spot_sensor_node started")

    odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
    scan_pub = rospy.Publisher('/scan', LaserScan, queue_size=10)

    rospy.loginfo("Spinning...")
    rospy.spin()

if __name__ == '__main__':
    try:
        initialize_node()
    except rospy.ROSInterruptException:
        pass
