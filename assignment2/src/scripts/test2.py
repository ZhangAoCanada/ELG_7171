#!/usr/bin/env python

## Subscribes to topic /scan of type LaserScan and display the minimum
## distance to an obstacle and their angle(s) relative to the scanner's
## reference frame

import numpy as np
import rospy
from sensor_msgs.msg import LaserScan



def callback(msg):
    ranges = msg.ranges

    # convert to numpy array to be able to use numpy functions
    npranges = np.array(ranges)

    # convert values out of range to 'NaN' to be ignored in calculation
    npranges[npranges > msg.range_max] = float('NaN')
    npranges[npranges < msg.range_min] = float('NaN')

    # compute minimum distance and its corresponding angles with respect to scanner's frame
    min_distance = np.nanmin(npranges)
    indices = np.reshape( np.argwhere(npranges == min_distance) , -1)

    # report the data
    rospy.loginfo('Min dist. [m] = %7.3f , Angles [ded] = %7.3f', min_distance, ((indices*msg.angle_increment)+msg.angle_min)*180.0/np.pi)


    
def laser_scan_reader():
    rospy.init_node('laser_scan_reader', anonymous=False)

    rospy.Subscriber('/scan', LaserScan, callback)

    rospy.spin()

if __name__ == '__main__':
    laser_scan_reader()