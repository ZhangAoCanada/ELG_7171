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
K_P = 0.1

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

    def ToDegree(self, theta):
        """
        Transfer the angle from radia to degree.
        """
        return theta / np.pi * 180.

    def GetRobotPos(self):
        """
        Find out the current position of the robot.
        """
        # read quaternion as the robot's current pose
        orientation_q = self.odom.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        # transfer to Euler's angles, output angles are in the range (-pi, pi)
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
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
        if len(ranges) > 1:
            # get the minimum range point with its angle to the robot
            range_min = np.min(ranges)
            if range_min > self.Lidar.range_max:
                range_min = None
            # find the corresponding index
            range_min_index = np.argmin(ranges)
            # calculate the angular position of the obstacle according to the range index
            object_angle = self.Lidar.angle_min + range_min_index * self.Lidar.angle_increment
            # transfer the angle to degrees
            object_angle_degree = object_angle
            # change the range to [-np.pi. np.pi]
            if object_angle_degree > np.pi:
                object_angle_degree -= 2 * np.pi
        else:
            # if nothing detected, set the default value to zero
            range_min = None
            object_angle_degree = None
        return range_min, object_angle_degree

    def GetNormalizedError(self, range_min):
        """
        Calculating normalized error using equation: e = d - l
        """
        # find out if there is an obstacle detected
        if range_min is not None:
            e = range_min - L
            # as assignment requests
            if e >= E_MAX:
                e_normalized = 1
            elif e <= - E_MAX:
                e_normalized = -1
            else:
                e_normalized = e / E_MAX
        # if no obstacles found, return None
        else:
            e = range_min
            e_normalized = range_min
        return e, e_normalized

    def GetVla(self, e):
        """
        Calculating the v_l^a according to the definition v_l^a = K_P * [e, 1 - e]^T
        """
        # proportional controller
        v = K_P * e
        w = K_P * (1 - np.abs(e))
        return v, w

    def GetVlw(self, v_a, w_a, alpha):
        """
        Transfer from v_l^a to the robot coordiate v_l^w.
        Note: H means Hormogenous transformation function.
        """
        # Transformation matrix^-1 * Rotation matrix
        H_11 = np.cos(THETA + alpha) + Y_L / X_L * np.sin(THETA + alpha)
        H_12 = -np.sin(THETA + alpha) + Y_L / X_L * np.cos(THETA + alpha)
        H_21 = 1 / X_L * np.sin(THETA + alpha)
        H_22 = 1 / X_L * np.cos(THETA + alpha)
        H_inverse = np.array([[H_11, H_12], [H_21, H_22]])
        # get v_l^a
        v_la = np.array([[v_a], [w_a]])
        v_new = np.matmul(H_inverse, v_la)
        # read the final results seperately
        v_w = v_new[0, 0]
        w_w = v_new[1, 0]
        return v_w, w_w

    def Navigation(self, v, w):
        """
        Navigate the robot with the specific velocity and angular velocity.
        """
        self.vel.linear.x = v
        self.vel.angular.z = w

    def StopMoving(self):
        """
        If target is not detected, stop.
        """
        self.vel.linear.x = 0.0
        self.vel.angular.z = 0.0

    def Run(self):
        """
        Main function, for running the code
        """
        while not rospy.is_shutdown():
            range_min, angle = self.GetObstaclePos()
            e_original, e_norm = self.GetNormalizedError(range_min)
            # if no obstacles found, stop there
            if e_norm is None:
                self.StopMoving()
                print("--------------------------------------------------")
                print("Nothing is being detected, please place something.")
                print("--------------------------------------------------")
                continue
                
            # calculate speeds
            v_a, w_a = self.GetVla(e_norm)
            v_w, w_w = self.GetVlw(v_a, w_a, angle)
            # print what are required in the assignment
            v_vector = np.array([v_a, w_a])
            angle_degree = self.ToDegree(angle)
            print("--------------------- state refreshing -----------------------")
            print("e:  {:.4f} m,\t e_bar:  {:.4f} m".format(e_original, e_norm))
            print("d:  {:.4f} m,\t alpha:  {:.4f} degrees".format(range_min, angle_degree))
            print("v_l in a coord:  [{:.4f}, {:.4f}] m".format(v_vector[0], v_vector[1]))
            print("v:  {:.4f}".format(v_w))
            print("w:  {:.4f}".format(w_w))
            print("x_L:  {:2f} m,\t y_L:  {:.2f} m,\t theta:  {:.2f}".format(X_L, Y_L, THETA))
            print("\n")

            # navigate the turtle
            self.Navigation(v_w, w_w)
            # publish change to the turtlesim
            self.pub.publish(self.vel)
            # sleep till the next commend sent
            self.rate.sleep()
            
if __name__ == "__main__":
    assignment3 = Assignment3()
    try:
        assignment3.Run()
    except rospy.ROSInternalException:
        pass