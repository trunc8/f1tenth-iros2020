#!/usr/bin/env python

# trunc8 did this

import rospy
import numpy as np
import message_filters
import tf
import math
import cv2
import os

from obstacle_detector.msg import CircleObstacle, Obstacles
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from scan_only_obstacles import Scan_Filter
from process_obstacles_as_circles import Obstacle_Processor

if __name__ == '__main__':
	try:
		rospy.init_node('obstacle_processor')
		obs_proc = Obstacle_Processor()
		sc_filter = Scan_Filter()
		while (rospy.get_time()==0):
			pass
		rospy.spin()

	except rospy.ROSInterruptException:
		rospy.loginfo("Node terminated")