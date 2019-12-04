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

class HuskyControl:
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
        # transfer to degrees
        roll = roll / np.pi * 180
        pitch = pitch / np.pi * 180
        yaw = yaw / np.pi * 180
        return (roll, pitch, yaw)

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
            object_angle_degree = object_angle / np.pi * 180
        else:
            # if nothing detected, set the default value to zero
            range_min = None
            object_angle_degree = None
        return range_min, object_angle_degree

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

    def GetTargetDist(self):
        """
        Find out the target position relative to the current robot position.
        """
        return np.sqrt((self.odom.pose.pose.position.x - self.target_x)**2 + \
                        (self.odom.pose.pose.position.y - self.target_y)**2)

    def GetTargetPos(self):
        """
        Find out the target orientation relative to the current robot position.
        """
        # get robot's current orientation
        roll, pitch, yaw = self.GetRobotPos()
        # find the angle to the destination is clockwise or counter clockwise
        sign = np.sign(self.target_y - self.odom.pose.pose.position.y).astype(np.float32)
        # set Euclidean vector [x, y] from current position to the destination
        vector_target = np.array([self.target_x - self.odom.pose.pose.position.x, \
                                    self.target_y - self.odom.pose.pose.position.y])
        # set the unit vector [1, 0] for calculating the direction angle.
        vector_x_dir = np.array([1., 0.])
        # calculate the magnitude of the vectors for further usage
        mag_vector_target = np.sqrt(np.sum(np.square(vector_target))).astype(np.float32)
        mag_vector_xdir = np.sqrt(np.sum(np.square(vector_x_dir))).astype(np.float32)
        # using the property of inner product:
        # cos(theta) = inner<vector1, vector2> / (||vector1|| * ||vector2||)
        target_angle_cos = np.dot(vector_target, vector_x_dir) / mag_vector_target / mag_vector_xdir
        target_angle = sign * np.arccos(target_angle_cos)
        # transfer the angle into degrees
        target_angle_degrees = target_angle / np.pi * 180
        target_orien_degrees = target_angle_degrees - yaw
        # if the orientation error is over 180 degrees, command the robot to turn the other way around
        if target_orien_degrees > 180:
            target_orien_degrees -= 360
        elif target_orien_degrees < -180:
            target_orien_degrees += 360
        return target_orien_degrees, target_angle_degrees

    def Navigation(self):
        """
        Navigate the robot to the destination.

        First move the orientation untill it is directing to the destination.
        Then moving the body to the destination.
        """
        # find the obstacle's position
        range_min, angle = self.GetObstaclePos()
        # find robot's current orientation
        roll, pitch, yaw = self.GetRobotPos()
        # find the distance from robot to the target
        target_dist = self.GetTargetDist()
        # calculate the target orientation to the robot and their orientation error
        target_orien, target_ang = self.GetTargetPos()
        # set alarming threshold as 1.35m (prepare for collision avoidance)
        if (range_min is not None) and (range_min < 1.35):
            # command the robot to stop forwarding and to start rotating when it 
            # reaches 1.1m
            if range_min < 1.1:
                # rotating the robot until it directing perpenticular to the obstacle
                if angle <= 0 and angle > -90:
                    # the reason I add -0.02 m/s to the linear speed is that
                    # I want to make it stop quickly when it approaches cone
                    self.vel.linear.x = -0.02
                    self.vel.angular.z = -0.2
                elif angle > 0 and angle < 90:
                    self.vel.linear.x = -0.02
                    self.vel.angular.z = 0.2
                # if the angle already meets the requirement, go straight forward
                else:
                    self.vel.linear.x = 0.2
                    self.vel.angular.z = 0.
            else:
            # if not in range 1.1m, explore straight forward to see what will happen
                self.vel.linear.x = 0.2
                self.vel.angular.z = 0.
        # if no collision alarming, command the robot to direct to the target
        elif target_dist >= 0.1:
            # change the orientation till it reaches the tolerance
            # positive means go counter clockwise, negative means clockwise
            if (target_orien >= 1) and (target_orien <= 180):
                self.vel.angular.z = 0.2
                self.vel.linear.x = 0.
            elif (target_orien <= -1) or (target_orien > 180):
                self.vel.angular.z = -0.2
                self.vel.linear.x = 0.
            # change the position till it reaches the distance tolerance
            else:
                self.vel.angular.z = 0.
                self.vel.linear.x = 0.2
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
                self.target_x, self.target_y = self.GetTarget()
                # if the destination is not in x >= 0 and y <= 11, ask user to re-input
                # till the right values found
                if (self.target_x < 0 or self.target_x > 10) or (self.target_y < 0 or self.target_y > 10):
                    print("-------------------------------------------------------")            
                    print("WARNING: Invalid Input, please reinput the destination.")
                    print("-------------------------------------------------------")            
                    self.target_x = None
                    self.target_y = None
                else:
                    print("Current Destination: [{}, {}]".format(self.target_x, self.target_y))
            else:
                # get all necessary parameters
                range_min, angle = self.GetObstaclePos()
                distance = self.GetTargetDist()
                orientation, target_orient = self.GetTargetPos()
                roll, pitch, yaw = self.GetRobotPos()

                if (range_min is not None) and (range_min < 1.6):
                    ###### special mode: print the information about collision avoidance #######
                    print("---------------------------------------------------------------------")            
                    print("WARNING: POTENTIAL COLLISION DETECTED, ENTER COLLISION AVOIDENCE MODE")
                    print("Obstacle Distance: \t {} m".format(range_min))
                    print("Obstacle Orientation: \t {} degrees".format(angle))
                    print("---------------------------------------------------------------------")
                else:
                    ################ print everthing according to the requirements #############
                    print("-------------------------- state freshing ---------------------------")
                    print("Robot's position: \t {} m".format([self.odom.pose.pose.position.x, \
                                                            self.odom.pose.pose.position.y]))
                    print("Target's position: \t {} m".format([self.target_x, self.target_y]))
                    print("Distance error: \t {} m".format(distance))
                    print("Robot's orientation: \t {} degrees".format(yaw))
                    print("Target's orientation: \t {} degrees".format(target_orient))
                    print("Orientation error: \t {} degrees".format(orientation))

                # navigate the turtle
                self.Navigation()
                # publish change to the turtlesim
                self.pub.publish(self.vel)
                # sleep till the next commend sent
                self.rate.sleep()
            

if __name__ == "__main__":
    assignment2 = HuskyControl()
    try:
        assignment2.Run()
    except rospy.ROSInternalException:
        pass
