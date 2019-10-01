#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class ControlTurtle:

    def __init__(self, final_x):
        rospy.init_node("controlpose", anonymous=True)
        self.pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
        self.sub = rospy.Subscriber("/turtle1/pose", Pose, self.Update)
        self.current_position = Pose()
        self.control_pos = Twist()
        self.rate = rospy.Rate(10)
        self.terminal_x = final_x

    def InitialControl(self):
        self.control_pos.linear.x = 0
        self.control_pos.linear.y = 0
        self.control_pos.linear.z = 0

        self.control_pos.angular.x = 0
        self.control_pos.angular.y = 0
        self.control_pos.angular.z = 0

    def Update(self, data):
        self.current_position = data

    def Main(self, ):
        self.InitialControl()

        while not rospy.is_shutdown():

            while self.current_position.x < self.terminal_x:
                self.control_pos.angular.z = 0.
                self.control_pos.linear.x = .2
                self.pub.publish(self.control_pos)
                self.rate.sleep()

            self.control_pos.linear.x = 0.
            self.control_pos.angular.z = 0.3

            self.pub.publish(self.control_pos)
            self.rate.sleep()

if __name__ == "__main__":

    final_x = 9

    try:
        control = ControlTurtle(final_x)
        control.Main()
    except rospy.ROSInternalException:
        pass

