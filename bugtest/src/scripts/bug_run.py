#!/usr/bin/env python2
"""
ELG7171 Assignment 2
Student Name:       Ao Zhang
Student Number:     0300039680
"""
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from tanbug import TangentBug

class BugTest:
    def __init__(self):
        """
        Initialization
        """
        # initialize (x, y) of destination as None for further usage
        self.target_x = None
        self.target_y = None
        # publishing node and subscribing node
        self.odom = Odometry()
        self.vel = Twist()
        self.Lidar = LaserScan()
        # intialize the node
        rospy.init_node("husky_nav", anonymous = False)
        # publish to /cmd_vel as assignment asks
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # subscribe to /odometry/filtered and /scan as assignment asks
        self.sub1 = rospy.Subscriber('/odometry/filtered', Odometry, self.UpdateOdom)
        self.sub2 = rospy.Subscriber('/scan', LaserScan, self.UpdateScan)
        # set up the commend sending frequency, which, as requirement, is 10Hz
        self.rate = rospy.Rate(10)

    def UpdateOdom(self, msg):
        """
        Update the message information from subscriber '/odometry/filtered'
        """
        self.odom = msg

    def UpdateScan(self, msg):
        """
        Update the message information from subscriber '/odometry/filtered'
        """
        self.Lidar = msg

    def GetRobotPos(self):
        """
        Find out the current position of the robot.
        """
        # read quaternion as the robot's current pose
        orientation_q = self.odom.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        # transfer to Euler's angles, output angles are in the range (-pi, pi)
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        # # transfer to degrees
        # roll = roll / np.pi * 180
        # pitch = pitch / np.pi * 180
        # yaw = yaw / np.pi * 180
        return roll, pitch, yaw

    def GetRobotInfo(self):
        """
        Function:
            Get all necessary information of current robot's position.
        """
        roll, pitch, yaw = self.GetRobotPos()
        return np.array([self.odom.pose.pose.position.x, self.odom.pose.pose.position.y, yaw])

    def GetObstaclePos(self):
        """
        Find out the minimum range w.r.t the obstacle and its angle w.r.t to the robot.
        """
        # read all points from laser scan
        ranges = self.Lidar.ranges
        # transfer to np.array
        ranges = np.array(ranges)
        # get all the outlier points to a very large value
        ranges = np.where(ranges < self.Lidar.range_min, self.Lidar.range_max + 10, ranges)
        ranges = np.where(ranges > self.Lidar.range_max, self.Lidar.range_max + 10, ranges)
        if len(ranges) != 0:
            # get all default angles
            angles = self.Lidar.angle_max - self.Lidar.angle_increment * np.arange(len(ranges))
        else:
            # if nothing detected, set the default value to zero
            ranges = None
            angles = None
        return ranges, angles

    def GetTarget(self):
        """
        Get the destination input from the input
        """
        print("Enter the x coordinate of the destination:")
        destination_x = raw_input()
        print("Enter the y coordinate of the destination:")
        destination_y = raw_input()
        # transfer the inputs to float and return them
        return float(destination_x), float(destination_y)

    def Run(self):
        """
        Main function, for running the code
        """
        while not rospy.is_shutdown():
            # get input of destination
            if (self.target_x is None) or (self.target_y is None):
                self.target_x, self.target_y = self.GetTarget()
                # if the destination is not in x >= 0 and y <= 11, ask user to re-input
                # till the right values found
                if (self.target_x < 0 or self.target_x > 10) or (self.target_y < -10 or self.target_y > 10):
                    print("-------------------------------------------------------")            
                    print("WARNING: Invalid Input, please reinput the destination.")
                    print("-------------------------------------------------------")            
                    self.target_x = None
                    self.target_y = None
                else:
                    print("Current Destination: [{}, {}]".format(self.target_x, self.target_y))
            else:
                ################################################################################################
                # get all necessary parameters
                goal = np.array([self.target_x, self.target_y])
                robot_pos = self.GetRobotInfo()
                ranges, angles = self.GetObstaclePos()

                if (ranges is not None) and (angles is not None):
                    ctrl = TangentBug(self.Lidar.range_max)
                    # obsts = ctrl.Continuity(ranges, angles, robot_pos[:2])
                    # print(len(obsts))
                    linear, omega = ctrl.MotionToGo(ranges, angles, goal, robot_pos)
                    print("=======================================")
                    # print([linear, omega])
                else:
                    linear = 0.
                    omega = 0.
                    print("---------------------------------------")
                    print("NO OBSTACLE DETECTED.")
                    print("---------------------------------------")

                ################################################################################################
                self.vel.linear.x = linear 
                self.vel.angular.z = omega
                self.pub.publish(self.vel)

                # sleep till the next commend sent
                self.rate.sleep()
            

if __name__ == "__main__":
    test = BugTest()
    try:
        test.Run()
    except rospy.ROSInternalException:
        pass





# def GetTargetPos(self):
#     """
#     Find out the target orientation relative to the current robot position.
#     """
#     # get robot's current orientation
#     roll, pitch, yaw = self.GetRobotPos()
#     # find the angle to the destination is clockwise or counter clockwise
#     sign = np.sign(self.target_y - self.odom.pose.pose.position.y).astype(np.float32)
#     # set Euclidean vector [x, y] from current position to the destination
#     vector_target = np.array([self.target_x - self.odom.pose.pose.position.x, \
#                                 self.target_y - self.odom.pose.pose.position.y])
#     # set the unit vector [1, 0] for calculating the direction angle.
#     vector_x_dir = np.array([1., 0.])
#     # calculate the magnitude of the vectors for further usage
#     mag_vector_target = np.sqrt(np.sum(np.square(vector_target))).astype(np.float32)
#     mag_vector_xdir = np.sqrt(np.sum(np.square(vector_x_dir))).astype(np.float32)
#     # using the property of inner product:
#     # cos(theta) = inner<vector1, vector2> / (||vector1|| * ||vector2||)
#     target_angle_cos = np.dot(vector_target, vector_x_dir) / mag_vector_target / mag_vector_xdir
#     target_angle = sign * np.arccos(target_angle_cos)
#     # transfer the angle into degrees
#     target_angle_degrees = target_angle / np.pi * 180
#     target_orien_degrees = target_angle_degrees - yaw
#     # if the orientation error is over 180 degrees, command the robot to turn the other way around
#     if target_orien_degrees > 180:
#         target_orien_degrees -= 360
#     elif target_orien_degrees < -180:
#         target_orien_degrees += 360
#     return target_orien_degrees, target_angle_degrees
