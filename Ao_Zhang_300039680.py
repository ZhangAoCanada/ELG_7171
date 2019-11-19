#!/usr/bin/env python
"""
ELG7171 Assignment 3
Student Name:       Ao Zhang
Student Number:     0300039680
"""
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion, quaternion_from_euler

"""
The translation and orientation information I received:

- Translation: [-0.032, 0.000, 0.172]
- Rotation: in Quaternion [0.000, 0.000, 0.000, 1.000]
            in RPY (radian) [0.000, -0.000, 0.000]
            in RPY (degree) [0.000, -0.000, 0.000]
"""
# hyper parameters settings
L = 1.
E_MAX = 0.3
X_L = -0.032
Y_L = 0.000
THETA = 0.000
K_P = 1.

class Assignment3(object):
    def __init__(self):
        """
        Initialization
        """
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
        # transfer to degrees
        # roll = roll / np.pi * 180
        # pitch = pitch / np.pi * 180
        # yaw = yaw / np.pi * 180
        return roll, pitch, yaw

    def GetObstaclePos(self):
        """
        Find out the minimum range w.r.t the obstacle and its angle w.r.t to the robot.
        """
        # read all points from laser scan
        ranges = self.Lidar.ranges
        # transfer to np.array
        ranges = np.array(ranges)
        # get all the outlier points to a very large value
        ranges = np.where(ranges < self.Lidar.range_min, self.Lidar.range_max + 1, ranges)
        ranges = np.where(ranges > self.Lidar.range_max, self.Lidar.range_max + 1, ranges)
        if len(ranges) != 0:
            # get the minimum range point with its angle to the robot
            range_min = np.min(ranges)
            # find the corresponding index
            range_min_index = np.argmin(ranges)
            # calculate the angular position of the obstacle according to the range index
            object_angle = self.Lidar.angle_min + range_min_index * self.Lidar.angle_increment
            # transfer the angle to degrees
            object_angle_degree = object_angle
            # object_angle_degree = object_angle / np.pi * 180
        else:
            # if nothing detected, set the default value to zero
            range_min = None
            object_angle_degree = None
        # if object_angle_degree > 180:
        #     object_angle_degree -= 360
        return range_min, object_angle_degree

    def GetNormalizedError(self, range_min):
        """
        Calculating normalized error using equation: e = d - l
        """
        if range_min is not None:
            e = range_min - L
            if e >= E_MAX:
                e_normalized = 1
            elif e <= - E_MAX:
                e_normalized = -1
            else:
                e_normalized = e / E_MAX
        else:
            e_normalized = range_min
        return e_normalized

    def GetVla(self, e):
        """
        Calculating the v_l^a according to the definition v_l^a = K_P * [e, 1 - e]^T
        """
        v = K_P * e
        w = K_P * (1 - np.abs(e))
        return v, w

    def GetVlw(self, v_a, w_a, alpha):
        """
        Transfer from v_l^a to the robot coordiate v_l^w.
        Note: H means Hormogenous transformation function.
        """
        H_11 = np.cos(THETA + alpha) + Y_L / X_L * np.sin(THETA + alpha)
        H_12 = -np.sin(THETA + alpha) + Y_L / X_L * np.cos(THETA + alpha)
        H_21 = 1 / X_L * np.sin(THETA + alpha)
        H_22 = 1 / X_L * np.cos(THETA + alpha)
        H_inverse = np.array([[H_11, H_12], [H_21, H_22]])
        v_la = np.array([[v_a], [w_a]])
        v_new = np.matmul(H_inverse, v_la)
        v_w = v_new[0, 0]
        w_w = v_new[1, 0]
        print("------------------------")
        print([v_a, w_a])
        print([v_w, w_w])
        return v_w, w_w

    # def Navigation(self):
         

    def Run(self):
        """
        Main function, for running the code
        """
        while not rospy.is_shutdown():
            range_min, angle_degree = self.GetObstaclePos()
            e_norm = self.GetNormalizedError(range_min)
            if e_norm is None:
                continue
            v_a, w_a = self.GetVla(e_norm)
            v_w, w_w = self.GetVlw(v_a, w_a, angle_degree)
            # print("---------------------")
            # print(angle_degree)
            # print([v_w, w_w])

            # # navigate the turtle
            # self.Navigation()
            # # publish change to the turtlesim
            # self.pub.publish(self.vel)
            # # sleep till the next commend sent
            # self.rate.sleep()
            

if __name__ == "__main__":
    assignment3 = Assignment3()
    try:
        assignment3.Run()
    except rospy.ROSInternalException:
        pass