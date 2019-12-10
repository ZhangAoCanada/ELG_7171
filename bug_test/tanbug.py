import numpy as np

class TangentBug(object):
    def __init__(self, laser_maximum):
        """
        Function:
            Initialization
        """
        self.continuity_threshold = 0.3
        self.range_max = laser_maximum
        self.scan_max = laser_maximum + 10

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
                if current_range <= self.range_max:
                    current_object.append([robot_pos[0] + current_range * np.cos(current_angle), \
                                            robot_pos[1] + current_range * np.sin(current_angle)])
                    previous_range = current_range
                    previous_angle = current_angle
                else:
                    previous_range = None
                    previous_angle = None
            elif current_range - previous_range > self.continuity_threshold or current_range > self.range_max:
                current_object.append([robot_pos[0] + previous_range * np.cos(previous_angle), \
                                        robot_pos[1] +  previous_range * np.sin(previous_angle)])
                all_objects.append(np.array(current_object))
                current_object = []
                if current_range <= self.range_max:
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

    def EuclideanDist(self, point1, point2):
        """
        Function:
            Calculation of Euclidean distance.
        """
        return np.sqrt(np.sum(np.square(point1 - point2)))

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
        if d1 <= d2:
            move_to = O1
        else:
            move_to = O2
        return move_to

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
            if current_range > self.range_max:
                current_range = self.range_max
            laser_seg = np.array([x, np.array([x[0] + current_range * np.cos(current_angle), \
                                            x[1] + current_range * np.sin(current_angle)])])
            current_d = self.LineSegDist(goal, laser_seg)
            if current_d <= d_reach:
                d_reach = current_d
        return d_reach

    def MotionToGo(self, ranges, angles, goal, robot_pose):
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
        obstacles = self.Continuity(ranges, angles, x, yaw)
        x_to_goal = np.array([x, goal])
        if len(obstacles) != 0:
            for obst_id in range(len(obstacles)):
                obstacle = obstacles[obst_id]
                intersection = self.Intersection(x_to_goal, obstacle)
                if intersection:
                    move_to_point = self.HeuristicDist(obstacle, x, goal)
                    d_follow = self.EuclideanDist(move_to_point, goal)
                    d_reach = self.FindDistReach(ranges, angles, goal, x, yaw)
                    print(obstacle)
                    print([d_follow, d_reach])
                    y_diff = move_to_point[1] - x[1]
                    theta = np.arcsin(y_diff / self.EuclideanDist(move_to_point, x))
                    # print([theta, yaw, theta-yaw])
                    if abs(theta - yaw) <= 0.2:
                        velocity = 0.1
                        omega = 0.
                    else:
                        omega = np.sign(theta - yaw) * 0.1
                        velocity = 0.
                    break
                velocity = 0.
                omega = 0.
        else:
            print("---------------------------------------")
            print("NO OBSTACLE DETECTED.")
            print("---------------------------------------")
            velocity = 0.
            omega = 0.
        return velocity, omega
