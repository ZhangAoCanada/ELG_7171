#!/usr/bin/env python
"""
ELG7171 Assignment 1
Student Name:       Ao Zhang
Student Number:     0300039680
"""
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class ControlTurtle:
    def __init__(self):
        """
        Initialization
        """
        # initialize (x, y) of destination as None for further usage
        self.target_x = None
        self.target_y = None
        # publishing node and subscribing node
        self.pose = Pose()
        self.vel = Twist()
        # intialize the node
        rospy.init_node("navigate_to_target", anonymous = False)
        self.pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/turtle1/pose', Pose, self.UpdatePose)
        # set up the commend sending frequency, which, as requirement, is 20Hz
        self.rate = rospy.Rate(20)

    def UpdatePose(self, msg):
        """
        Update the message information from subscriber '/turtle1/pose'
        """
        self.pose = msg

    def GetDestination(self):
        """
        Get the destination input from the input
        """
        print("Enter the x coordinate of the destination:")
        destination_x = raw_input()
        print("Enter the y coordinate of the destination:")
        destination_y = raw_input()
        # transfer the inputs to float and return them
        return float(destination_x), float(destination_y)

    def GetDistance(self):
        """
        Get the Euclideam distance from current position to the destination
        """
        return np.sqrt((self.pose.x - self.target_x)**2 + \
                        (self.pose.y - self.target_y)**2)

    def GetOrientation(self):
        """
        Get the orienation difference between current orientation and the direction 
        to the destination

        Note:   positive sign         ->          counter clockwise
                negative sign         ->          clockwise
        """
        # find the angle to the destination is clockwise or counter clockwise
        sign = np.sign(self.target_y - self.pose.y).astype(np.float32)
        # set Euclidean vector [x, y] from current position to the destination
        vector_target = np.array([self.target_x - self.pose.x, \
                                self.target_y - self.pose.y])
        # set the unit vector [1, 0] for calculating the direction angle.
        vector_x_dir = np.array([1., 0.])
        # calculate the magnitude of the vectors for further usage
        mag_vector_target = np.sqrt(np.sum(np.square(vector_target))).astype(np.float32)
        mag_vector_xdir = np.sqrt(np.sum(np.square(vector_x_dir))).astype(np.float32)
        # using the property of inner product:
        # cos(theta) = inner<vector1, vector2> / (||vector1|| * ||vector2||)
        target_angle_cos = np.dot(vector_target, vector_x_dir) / mag_vector_target / mag_vector_xdir
        
        # transfer the counter clockwise to clockwise, since the Pose node returns angle 
        # values from (0, 2*pi)
        if sign >= 0:
            target_angle = np.arccos(target_angle_cos)
        else:
            target_angle = 2*np.pi - np.arccos(target_angle_cos)

        return (target_angle - self.pose.theta)

    def Navigate(self, distance, orientation):
        """
        Navigate the turtle to the destination.

        First move the orientation untill it is directing to the destination.
        Then moving the body to the destination.
        """
        # change the orientation till it reaches the tolerance
        # positive means go counter clockwise, negative means clockwise
        if orientation >= 0.05:
            self.vel.angular.z = 0.1
            self.vel.linear.x = 0.
        elif orientation <= -0.05:
            self.vel.angular.z = -0.1
            self.vel.linear.x = 0.
        # change the position till it reaches the distance tolerance
        elif distance >= 0.1:
            self.vel.angular.z = 0.
            self.vel.linear.x = 0.1
        # once reach the destination, stop the turtle
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
        """
        Main function, for running the code
        """
        while not rospy.is_shutdown():
            # get input of destination
            if (self.target_x is None) or (self.target_y is None):
                self.target_x, self.target_y = self.GetDestination()
                # if the destination is not in x >= 0 and y <= 11, ask user to re-input
                # till the right values found
                if self.target_x < 0 or self.target_y > 11:
                    print("WARNING: Invalid Input, please reinput the destination.")
                    self.target_x = None
                    self.target_y = None
                else:
                    print("Current Destination: [{}, {}]".format(self.target_x, self.target_y))
            else:
                # get distance and orientation errors
                distance = self.GetDistance()
                orientation = self.GetOrientation()
                
                # transfer radian to degree (as requirement says)
                orientation_degrees = orientation / np.pi * 180
                ################ print distance and orientation errors #############
                print("Distance error: \t {} m".format(distance))
                print("Orientation error: \t {} degrees".format(orientation_degrees))

                # navigate the turtle
                self.Navigate(distance, orientation)
                # publish change to the turtlesim
                self.pub.publish(self.vel)

                # sleep till the next commend sent
                self.rate.sleep()

if __name__ == "__main__":
    try:
        navigation = ControlTurtle()
        navigation.Run()
    except rospy.ROSInternalException:
        pass

    