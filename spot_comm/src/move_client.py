#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped

def sdkCall(data):
    rospy.loginfo(rospy.get_caller_id() + "Pose recieved")
    
def listener():
    rospy.init_node('moveClient', anonymous=True)
    rospy.Subscriber("move_base_simple/goal", PoseStamped, sdkCall)
    rospy.spin()

if __name__ == '__main__':
    listener()
