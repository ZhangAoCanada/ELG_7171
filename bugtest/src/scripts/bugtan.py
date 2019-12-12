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
rosrun tf tf_echo /odom /base_laser

- Translation: [0.337, 0.000, 0.308]
- Rotation: in Quaternion [1.000, 0.000, 0.000, 0.000]
            in RPY (radian) [3.142, -0.000, 0.000]
            in RPY (degree) [180.000, -0.000, 0.003]
"""
# hyper parameters settings
ROTATION_TOLERANCE = 0.02
CONTINUITY_MAX = 1.2
CONTINUITY_THRESHOLD = 0.5
CONFIG_DISTANCE = 1
ROBOT_SIZE = 1

MLINE_MEET_THRESHOLD = 0.2
CLOSER_THRESHOLD = 0.5
GOAL_REACHED_THRESHOLD = 0.5
Q_H_REVISIT_THRESHOLD = 0.5
Q_L_MEET = 0.3
L = 1.
E_MAX = 0.3
X_L = 0.337
Y_L = 0.000
THETA = 0.00
K_P = 0.2

class TangentBugAlgorithm(object):
    def __init__(self):
        """
        Initialization
        """
        # initialize (x, y) of destination as None for further usage
        self.target_x = None
        self.target_y = None
        self.q_hit = None
        self.explore = True
        self.whole_obstacle_visited = False
        self.leave = False
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
        ranges = np.where(ranges < self.Lidar.range_min, self.Lidar.range_max + 10., ranges)
        ranges = np.where(ranges > self.Lidar.range_max, self.Lidar.range_max + 10., ranges)
        if len(ranges) > 1:
            # get the minimum range point with its angle to the robot
            range_ind = np.arange(len(ranges))
            range_min = np.min(ranges)
            if range_min > self.Lidar.range_max:
                range_min = None
            # calculate the angular position of the obstacle according to the range index
            angles = self.Lidar.angle_max - range_ind * self.Lidar.angle_increment
            range_min_index = np.argmin(ranges)
            object_angle = angles[range_min_index]
        else:
            # if nothing detected, set the default value to zero
            range_min = None
            object_angle = None
            ranges = None
            angles = None
        return ranges, angles, range_min, object_angle

    ##############################################################################
    #                       Boundary Following Behavior                          #
    ##############################################################################
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
        H_22 = 1 /X_L * np.cos(THETA + alpha)
        H_inverse = np.array([[H_11, H_12], [H_21, H_22]])
        # get v_l^a
        v_la = np.array([[v_a], [w_a]])
        v_new = np.matmul(H_inverse, v_la)
        # read the final results seperately
        v_w = v_new[0, 0]
        w_w = v_new[1, 0]
        return v_w, w_w

    def BoundaryFollowing(self, range_min, angle_min):
        """
        Calculate the linear and angular velocity for boundary following.
        """
        e_original, e_norm = self.GetNormalizedError(range_min)
        # if no obstacles found, stop there
        if e_norm is None:
            v_w = 0.
            w_w = 0.
            print("--------------------------------------------------")
            print("Nothing is being detected, please place something.")
            print("--------------------------------------------------")
        else:
            # calculate speeds
            v_a, w_a = self.GetVla(e_norm)
            v_w, w_w = self.GetVlw(v_a, w_a, angle_min)
        return v_w, w_w

    ##############################################################################
    #                              Geometric properties                          #
    ##############################################################################
    def EuclideanDist(self, point1, point2):
        """
        Calculate the Euclidean Distance between two points
        """
        return np.sqrt(np.sum(np.square(point1 - point2)))

    def Continuity(self, ranges, angles, robot_pos, yaw):
        """
        Function:
            Find out how many line segments there are for each obstacle.

        Args:
            ranges      ->      laser ray's ranges
            angles      ->      laser ray's angles
            robot_pos   ->      robot current position

        Returns:
            all_objects ->      [ [O1, O2], [O3, O4], ... ]
        """
        all_objects = []
        current_object = []
        for ind in range(len(ranges)):
            current_range = ranges[ind]
            current_angle = angles[ind] + yaw
            if len(current_object) == 0:
                if current_range <= self.Lidar.range_max:
                    current_object.append([robot_pos[0] + current_range * np.cos(current_angle), \
                                            robot_pos[1] + current_range * np.sin(current_angle)])
                    previous_range = current_range
                    previous_angle = current_angle
                else:
                    previous_range = None
                    previous_angle = None
            elif (current_range - previous_range) > CONTINUITY_THRESHOLD or current_range > self.Lidar.range_max or ind == len(ranges)-1:
                current_object.append([robot_pos[0] + previous_range * np.cos(previous_angle), \
                                        robot_pos[1] +  previous_range * np.sin(previous_angle)])
                all_objects.append(np.array(current_object))
                current_object = []
                if current_range <= self.Lidar.range_max:
                    current_object.append([robot_pos[0] + current_range * np.cos(current_angle), \
                                            robot_pos[1] +  current_range * np.sin(current_angle)])
            else:
                previous_range = current_range
                previous_angle = current_angle
        return all_objects

    def Intersection(self, line_seg1, line_seg2):
        """
        Function:
            Define two line segments intersect or not.

        Args:
            line_seg1       ->      [O1, O2]
            line_seg2       ->      [O3, O4]

        Returns:
            intersection    ->      boolean_mask for telling whether 2 line segments intersect
        """
        pt_a = line_seg1[0]
        pt_b = line_seg1[1]
        pt_c = line_seg2[0]
        pt_d = line_seg2[1]
        delta = (pt_b[0] - pt_a[0]) * (pt_d[1] - pt_c[1]) - (pt_b[1] - pt_a[1]) * (pt_d[0] - pt_c[0])
        n_r = (pt_a[1] - pt_c[1]) * (pt_d[0] - pt_c[0]) - (pt_a[0] - pt_c[0]) * (pt_d[1] - pt_c[1])
        n_s = (pt_a[1] - pt_c[1]) * (pt_b[0] - pt_a[0]) - (pt_a[0] - pt_c[0]) * (pt_b[1] - pt_a[1])
        if delta != 0:
            r = n_r / delta
            s = n_s / delta
            if r >= 0 and r <= 1 and s >= 0 and s <= 1:
                intersection = True
            else:
                intersection = False
        else:
            intersection = False
        return intersection

    def LineSegDist(self, point, line_seg):
        """
        Function:
            Calculate the distance between the point and the line segment.

        Args:
            point       ->      given point [x, y]
            line_seg    ->      line segment [v, w]

        Returns:
            d           ->      distance
        """
        v = line_seg[0]
        w = line_seg[1]
        vp = point - v
        vw = w - v
        t_q = np.sum(vp * vw) / np.sum(np.square(vw))
        t_q = np.maximum(0, np.minimum(t_q, 1))
        q_bar = v + t_q * vw
        # d = np.sqrt(np.sum(np.square(point - q_bar)))
        d = self.EuclideanDist(point, q_bar)
        return d

    def HeuristicDist(self, line_seg, x, goal):
        """
        Function:
            Calculation of heuristicDist
        
        Args:
            line_seg        ->      line segment of the obstacle [O1, O2]
            goal            ->      [goal_x, goal_y]
            x               ->      robot's position, normally, [0, 0]
        """
        O1 = line_seg[0]
        O2 = line_seg[1]
        d1 = self.EuclideanDist(O1, x) + self.EuclideanDist(O1, goal)
        d2 = self.EuclideanDist(O2, x) + self.EuclideanDist(O2, goal)
        O1_new, O2_new = self.PntToConfig(O1, O2)
        if d1 <= d2:
            move_to = O1_new
        else:
            move_to = O2_new
        return move_to

    def PntToConfig(self, point1, point2):
        """
        Move to the point with certain range in order to avoid collision
        """
        mid_point = (point1 + point2) / 2.
        mid_point[0] = 0.
        diff_point1 = point1 - mid_point
        theta_point1 = np.arcsin(diff_point1[1] / np.sqrt(np.sum(np.square(diff_point1))))
        new_point1 = point1 + np.array([-CONFIG_DISTANCE*np.cos(theta_point1), CONFIG_DISTANCE*np.sin(theta_point1)])

        diff_point2 = point2 - mid_point
        theta_point2 = np.arcsin(diff_point2[1] / np.sqrt(np.sum(np.square(diff_point2))))
        new_point2 = point2 + np.array([-CONFIG_DISTANCE*np.cos(theta_point2), CONFIG_DISTANCE*np.sin(theta_point2)])
        return new_point1, new_point2

    ##############################################################################
    #                           Tangent Bug Alogrithm                            #
    ##############################################################################
    def FindDistReach(self, ranges, angles, goal, x, yaw):
        """
        Function:
            Find N_reach with its distance.
        
        Args:
            ranges          ->          all ranges from Lidar
            angles          ->          all angles from Lidar
            goal            ->          [goal_x, goal_y]
        """
        d_reach = 100
        for i in range(len(ranges)):
            current_range = ranges[i]
            current_angle = angles[i] + yaw
            if current_range > self.Lidar.range_max:
                current_range = self.Lidar.range_max
            laser_seg = np.array([x, np.array([x[0] + current_range * np.cos(current_angle), \
                                            x[1] + current_range * np.sin(current_angle)])])
            current_d = self.LineSegDist(goal, laser_seg)
            if current_d <= d_reach:
                d_reach = current_d
        return d_reach

    def FindDistFollow(self, ranges, angles, goal, x, yaw):
        """
        Function:
            Find N_reach with its distance.
        
        Args:
            ranges          ->          all ranges from Lidar
            angles          ->          all angles from Lidar
            goal            ->          [goal_x, goal_y]
        """
        d_reach = 100
        for i in range(len(ranges)):
            current_range = ranges[i]
            current_angle = angles[i] + yaw
            if current_range > self.Lidar.range_max:
                current_range = 0.
            laser_seg = np.array([x, np.array([x[0] + current_range * np.cos(current_angle), \
                                            x[1] + current_range * np.sin(current_angle)])])
            current_d = self.LineSegDist(goal, laser_seg)
            if current_d <= d_reach:
                d_reach = current_d
        return d_reach

    def DistPntLine(self, point, line):
        """
        Calculate the distance between a point to a line.
        """
        vector = np.array([point[0], point[1], 1]).T
        distance = np.matmul(line, vector) / np.sqrt(line[0]**2 + line[1]**2)
        return distance

    def ObstacleBLocking(self, robot_pose, ranges, angles):
        """
        Find out if there is an obstacle blocking the way.
        """
        all_obstacles = self.Continuity(ranges, angles, robot_pose[:2], robot_pose[-1])
        print("size of ranges:", len(ranges))
        print("how many obstacles:", len(all_obstacles))
        x_to_goal = np.array([robot_pose[:2], np.array([self.target_x, self.target_y])])
        # xmin_to_goal = np.array([robot_pose[:2] + ROBOT_SIZE*np.array([np.cos(robot_pose[-1]), 
        #                             ROBOT_SIZE*np.sin(robot_pose[-1])]), np.array([self.target_x, self.target_y])])
        # xmax_to_goal = np.array([robot_pose[:2] - ROBOT_SIZE*np.array([np.cos(robot_pose[-1]), 
        #                             ROBOT_SIZE*np.sin(robot_pose[-1])]), np.array([self.target_x, self.target_y])])
        intersection = False
        obstacle = None
        for obstacle_id in range(len(all_obstacles)):
            obstacle = all_obstacles[obstacle_id]
            intersection = self.Intersection(x_to_goal, obstacle)
            # intersection_min = self.Intersection(xmin_to_goal, obstacle)
            # intersection_max = self.Intersection(xmax_to_goal, obstacle)
            # intersection = intersection and intersection_min and intersection_max
            if intersection:
                break
        return intersection, obstacle
    
    def GoalReached(self, robot_pose):
        """
        Find out whether the goal is reached.
        """
        robot_location = robot_pose[:2]
        target_location = np.array([self.target_x, self.target_y])
        distance = self.EuclideanDist(robot_location, target_location)
        if distance <= GOAL_REACHED_THRESHOLD:
            goal_reached = True
        else:
            goal_reached = False
        return goal_reached

    def GetTargetPos(self, robot_pose):
        """
        Find out the target orientation relative to the current robot position.
        """
        robot_x = robot_pose[0]
        robot_y = robot_pose[1]
        yaw = robot_pose[-1]
        sign = np.sign(self.target_y - robot_y).astype(np.float32)
        vector_target = np.array([self.target_x - robot_x, self.target_y - robot_y])
        vector_x_dir = np.array([1., 0.])
        mag_vector_target = np.sqrt(np.sum(np.square(vector_target))).astype(np.float32)
        mag_vector_xdir = np.sqrt(np.sum(np.square(vector_x_dir))).astype(np.float32)
        # cos(theta) = inner<vector1, vector2> / (||vector1|| * ||vector2||)
        target_angle_cos = np.dot(vector_target, vector_x_dir) / mag_vector_target / mag_vector_xdir
        target_angle = sign * np.arccos(target_angle_cos)
        target_orientation = target_angle - yaw
        if target_orientation > np.pi:
            target_orientation -= 2 * np.pi
        elif target_orientation < -np.pi:
            target_orientation += 2 * np.pi
        return target_orientation

    def MoveToTarget(self, robot_pose, orientation_err):
        """
        Move straight forward to the target
        """
        print("orien_err:", orientation_err)
        if (orientation_err >= ROTATION_TOLERANCE) and (orientation_err <= np.pi):
            v = 0.
            w = 0.2
        elif (orientation_err <= -ROTATION_TOLERANCE) and (orientation_err >= -np.pi):
            v = 0.
            w = -0.2
        else:
            v = 0.2
            w = 0.
        return v, w

    def MotionToGo(self, obstacle, goal, robot_pose):
        """
        Function:
            Implement motion to go.

        Args:
            ranges      ->      laser ray's ranges
            angles      ->      laser ray's angles
            goal        ->      [goal_x, goal_y]
            x           ->      robot's positio, normally [0, 0]
        """
        x = robot_pose[:2]
        yaw = robot_pose[-1]
        x_to_goal = np.array([x, goal])
        move_to_point = self.HeuristicDist(obstacle, x, goal)
        print(obstacle)
        print(move_to_point)
        y_diff = move_to_point[1] - x[1]
        theta = np.arcsin(y_diff / self.EuclideanDist(move_to_point, x))
        # print([theta, yaw, theta-yaw])
        if abs(theta - yaw) <= 0.2:
            velocity = 0.2
            omega = 0.
        else:
            omega = np.sign(theta - yaw) * 0.2
            velocity = 0.
        return velocity, omega

    def TangentBug(self, robot_pose, ranges, angles, range_min, angle_min):
        """
        Self-implementation of Bug 1
        """
        orientation_err = self.GetTargetPos(robot_pose)
        if self.GoalReached(robot_pose):
            v = 0.
            w = 0.
            self.target_x = None
            self.target_y = None
            print("----------------------------------------------------")
            print("TARGET REACHED, PLEASE ENTER A NEW TARGET.")
            print("----------------------------------------------------")
            print("\n")
        else:
            blocking, obstacle = self.ObstacleBLocking(robot_pose, ranges, angles)
            if range_min is None or (not blocking and self.q_hit is None):
                v, w = self.MoveToTarget(robot_pose, orientation_err)
            else:
                if self.q_hit is None:
                    self.q_hit = robot_pose[:2]
                
                v, w = self.MotionToGo(obstacle, \
                                np.array([self.target_x, self.target_y]), robot_pose)

    #             v, w = self.BoundaryFollowing(range_min, angle_min)
    #             mline_meet = self.MlineMeet(robot_pose)
                
    #             if self.explore:
    #                 if self.EuclideanDist(robot_pose[:2], self.q_hit) > Q_H_REVISIT_THRESHOLD + 0.2:
    #                     self.explore = False
    #             else:
    #                 if mline_meet:
    #                     if not blocking:
    #                         d_x_goal = self.EuclideanDist(robot_pose[:2], np.array([self.target_x, self.target_y]))
    #                         d_qH_goal = self.EuclideanDist(self.q_hit, np.array([self.target_x, self.target_y]))
    #                         if (d_qH_goal - d_x_goal) >= CLOSER_THRESHOLD:
    #                             self.leave = True
    #                 if self.EuclideanDist(robot_pose[:2], self.q_hit) <= Q_H_REVISIT_THRESHOLD:
    #                     self.whole_obstacle_visited = True

    #             print("Whether mline met:", mline_meet)
    #             print("whether leave:", self.leave)

    #             if self.leave:
    #                 self.q_hit = None
    #                 self.explore = True
    #                 self.leave = False
            
    #             if self.whole_obstacle_visited:
    #                 self.q_hit = None
    #                 self.explore = True
    #                 self.leave = False
    #                 self.whole_obstacle_visited = False
    #                 v = 0.
    #                 w = 0.
    #                 self.target_x = None
    #                 self.target_y = None
    #                 self.m_line = None
    #                 print("---------------------------------------------------------")
    #                 print("NO SOLUTION EXIST, ENTER A NEW DESTINATION AND TRY AGAIN.")
    #                 print("---------------------------------------------------------")
    #                 print("\n")
        return v, w

    ##############################################################################
    #                                  Normal Part                               #
    ##############################################################################
    def Navigation(self, v, w):
        """
        Navigate the robot with the specific velocity and angular velocity.
        """
        self.vel.linear.x = v
        self.vel.angular.z = w

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
                robot_pose = self.GetRobotInfo()
                ranges, angles, range_min, angle_min = self.GetObstaclePos()
                v, w = self.TangentBug(robot_pose, ranges, angles, range_min, angle_min)
                print("---------------------- keep freshing the state --------------------")
                # print(self.explore)

                self.Navigation(v, w)
                self.pub.publish(self.vel)
                self.rate.sleep()
            
if __name__ == "__main__":
    test = TangentBugAlgorithm()
    try:
        test.Run()
    except rospy.ROSInternalException:
        pass