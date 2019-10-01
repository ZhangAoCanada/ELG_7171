#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

def PoseControl():
    rospy.init_node("controlpose", anonymous=True)
    pub = rospy.Publisher("/turtle1/cmd_vel", Twist(), queue_size=10)
    current_position = Twist()

    speed = .2
    rate = rospy.Rate(10)

    total_distance = 9

    while not rospy.is_shutdown():
        time_start = rospy.get_time()
        current_distance = 0

        while (current_distance < total_distance):
            current_position.linear_velocity = .2
            current_position.angular_velocity = 0.
            time_duration = rospy.get_time() - time_start
            current_distance = time_duration * current_position.linear_velocity
            # current_position.x = current_distance
        
        current_position.linear_velocity = 0

        pub.publish(current_position)
        rate.sleep()

if __name__ == "__main__":
    try:
        PoseControl()
    except rospy.ROSInternalException:
        pass

