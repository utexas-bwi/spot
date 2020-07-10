#!/usr/bin/env python3

import actionlib
import rospy
import spot_comm.msg

from geometry_msgs.msg import Pose2D, PointStamped
from sensor_msgs.msg import NavSatFix
from spot_comm.client import RobotClient


class RobotNavigationInterface:

    def __init__(self, client, mutex,robot_prefix="smads_platform"):
        self.client = client
        self.mutex = mutex
        self.robot_prefix = robot_prefix

        # Register Navigation Endpoint for map goals
        rospy.Subscriber("/smads_waypoint/map_goal", Pose2D, self.map_waypoint_cb)
        
        # Register Navigation Endpoint for GPS goals
        rospy.Subscriber("/smads_waypoint/gps_goal", Pose2D, self.gps_waypoint_cb)
        
        rospy.Subscriber("/smads/gps_to_map/result", PointStamped, self.gps_translator_cb)
        self.gps_translator_pub = rospy.Publisher("/smads/gps_to_map/input", NavSatFix, queue_size=1)

    #
    # Note this is kind of dangerous, since it assumes that all gps translations it receives
    # correspond to data sent from this interface. 
    # TODO make gps translator a library, not ROS interface
    #
    def gps_translator_cb(self, data):
        pose = Pose2D()
        pose.x = data.point.x
        pose.y = data.point.y

        # send goal
        self.send_client_goal(pose)

    def gps_waypoint_cb(self, data):
        nav = NavSatFix()
        nav.latitude = data.x
        nav.longitude = data.y
        self.gps_translator_pub.publish(nav)

    def map_waypoint_cb(self, data):
        self.send_client_goal(data)

    def send_client_goal(self, goal):
        
        # publish info to the console for the user
        rospy.loginfo('%s: Executing, creating sending waypoint goal of pose %s' % ("RobotNavigationInterface", goal))
        
        # TODO- this should poll the execution, update the feedback message and check that the cmd hasn't been preempted
        # and check if it has succeeded. simply set the waypoint for now
        with self.mutex:
            self.client.set_waypoint(goal)
          
    def start(self):
        rospy.loginfo('robot_navigation_interface active')
        rospy.spin()
        rospy.loginfo('robot_navigation_interface shutting down')

