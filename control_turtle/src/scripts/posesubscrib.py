#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

def Callback(data):
    rospy.loginfo(data)

def PoseSubscriber():
    rospy.init_node("posesub", anonymous= True)
    sub = rospy.Subscriber("/turtle1/pose", Pose, Callback)

    rospy.spin()

if __name__ == "__main__":
    try:
        PoseSubscriber()
    except rospy.ROSInternalException:
        pass


    
