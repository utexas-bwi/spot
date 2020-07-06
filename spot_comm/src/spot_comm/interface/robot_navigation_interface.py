#!/usr/bin/env python3

import actionlib
import rospy
import spot_comm.msg

from geometry_msgs.msg import PoseStamped
from spot_comm.client import RobotClient


class RobotNavigationInterface:

    def __init__(self, client, mutex,robot_prefix="smads_platform"):
        self.client = client
        self.mutex = mutex
        self.robot_prefix = robot_prefix

        # Register Navigation Endpoint
        self._action_name = "smads_waypoint"
        self._as = actionlib.SimpleActionServer(self._action_name, spot_comm.msg.WaypointAction, execute_cb=self.waypoint_action_cb, auto_start = False)
        self._as.start()
 
    def waypoint_cmd_cb(self, data):
        self.waypoint_cmd = data 

    def waypoint_action_cb(self, goal):
        
        # publish info to the console for the user
        rospy.loginfo('%s: Executing, creating sending waypoint goal of pose %s' % (self._action_name, goal.pose))
        
        # TODO- this should poll the execution, update the feedback message and check that the cmd hasn't been preempted
        # and check if it has succeeded. simply set the waypoint for now
        with self.mutex:
            self.client.set_waypoint_cmd(self, pose)
          
    def start(self):
        rospy.loginfo('robot_navigation_interface active')
        rospy.spin()
        rospy.loginfo('robot_navigation_interface shutting down')

