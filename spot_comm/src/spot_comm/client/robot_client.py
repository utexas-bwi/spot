#####################################################################
#
# RobotClient is an abstract class that offers common end points 
# to needed robot actions.
# Each new platform will need to extend RobotClient with that
# platform's particular method of information communication
#
#####################################################################


from abc import ABC, abstractmethod 
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image


class RobotClient(ABC): 
  
    ##############################
    # Abstract Retrieval Methods #
    ##############################
    def get_odom(self): 
        return Odometry()
    
    def get_rgb_vision(self): 
        return [Image()]

    def get_laserscan(self):
        return LaserScan()

    def get_robot_pose(self):
        return PoseWithCovarianceStamped()

    def get_robot_status(self):
        pass

    ##############################
    # Abstract Commander Methods #
    ##############################

    def set_waypoint(self, point):
        pass

    def set_estop_behavior(self):
        pass

    def start_robot(self):
        pass

    def stop_robot(self):
        pass

    def connect(self):
        pass

