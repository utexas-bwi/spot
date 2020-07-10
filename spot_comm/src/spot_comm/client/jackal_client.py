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
from amrl_msgs.msg import Pose2Df
from geometry_msgs.msg import PoseStamped

class JackalClient(RobotClient):

    def __init__(self, init_node=False):
        if init_node:
            rospy.init_node('jackal_ros_client_interface', anonymous=False)

        self.cur_odom = Odometry()
        self.cur_scan = LaserScan()

        rospy.Subscriber("/jackal_velocity_controller/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/velodyne_2dscan_high_beams", LaserScan, self.scan_callback)
        
        self.movebase_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped)

        # For new navigation, use publisher below
        #self.movebase_pub = rospy.Publisher("/move_base_simple/goal", Pose2Df)

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

    def set_waypoint(self, goal_pose):
        p = PoseStamped()
        p.pose.position.x = goal_pose.x
        p.pose.position.y = goal_pose.y
        p.header.frame_id = "map"
        self.movebase_pub.publish(p)
    
    """
    For new Navigation use below method instead
    def set_waypoint(self, goal_pose):
        p = Pose2Df()
        p.x = goal_pose.x
        p.y = goal_pose.y
        p.theta = goal_pose.theta

        self.movebase_pub.publish(p)
    """

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
