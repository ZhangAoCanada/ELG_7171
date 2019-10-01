#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class ControlTurtle:
    def __init__(self):
        self.target_x = None
        self.target_y = None

        self.pose = Pose()
        self.vel = Twist()

        rospy.init_node("navigate_to_target", anonymous = False)
        self.pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/turtle1/pose', Pose, self.UpdatePose)

        self.rate = rospy.Rate(20)

    def UpdatePose(self, msg):
        self.pose = msg

    def GetDestination(self):
        print("Enter the x coordinate of the destination:")
        destination_x = raw_input()
        print("Enter the y coordinate of the destination:")
        destination_y = raw_input()
        return float(destination_x), float(destination_y)

    def GetDistance(self):
        return np.sqrt((self.pose.x - self.target_x)**2 + \
                        (self.pose.y - self.target_y)**2)

    def GetOrientation(self):
        sign = np.sign(self.target_y - self.pose.y).astype(np.float32)
        vector_target = np.array([self.target_x - self.pose.x, \
                                self.target_y - self.pose.y])
        vector_x_dir = np.array([1., 0.])
        mag_vector_target = np.sqrt(np.sum(np.square(vector_target))).astype(np.float32)
        mag_vector_xdir = np.sqrt(np.sum(np.square(vector_x_dir))).astype(np.float32)
        target_angle_cos = np.dot(vector_target, vector_x_dir) / mag_vector_target / mag_vector_xdir

        if sign >= 0:
            target_angle = np.arccos(target_angle_cos)
        else:
            target_angle = 2*np.pi - np.arccos(target_angle_cos)

        return (target_angle - self.pose.theta)

    def Navigate(self, distance, orientation):
        if orientation >= 0.1:
            self.vel.angular.z = 0.1
            self.vel.linear.x = 0.
        elif orientation <= -0.1:
            self.vel.angular.z = -0.1
            self.vel.linear.x = 0.
        elif distance >= 0.1:
            self.vel.angular.z = 0.
            self.vel.linear.x = 0.1
        else:
            self.vel.angular.z = 0.
            self.vel.linear.x = 0.
            self.target_x = None
            self.target_y = None
            print("----------------------------------------------")
            print("target reached, please enter new destination")
            print("----------------------------------------------")
            print("\n")

    def Run(self):

        while not rospy.is_shutdown():

            if (self.target_x is None) or (self.target_y is None):
                self.target_x, self.target_y = self.GetDestination()
                if self.target_x < 0 or self.target_y > 11:
                    print("WARNING: Invalid Input, please reinput the destination.")
                    self.target_x = None
                    self.target_y = None
                else:
                    print("Current Destination: [{}, {}]".format(self.target_x, self.target_y))
            else:
                distance = self.GetDistance()
                orientation = self.GetOrientation()
                self.Navigate(distance, orientation)
                self.pub.publish(self.vel)
                print("Distance between target and current position:")
                print(distance)
                print("Orientation between target and current position:")
                print(orientation)

                # self.vel.angular.z = 0.1
                # self.pub.publish(self.vel)
                # print(self.pose.theta)
                self.rate.sleep()


if __name__ == "__main__":
    try:
        navigation = ControlTurtle()
        navigation.Run()
    except rospy.ROSInternalException:
        pass

    