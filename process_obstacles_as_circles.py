#!/usr/bin/env python

# trunc8 did this

import rospy
from obstacle_detector.msg import CircleObstacle, Obstacles
from geometry_msgs.msg import Point
import numpy as np

class Obstacle_Processor:
	def __init__(self):
		self.processed_obstacles_msg = Obstacles()
		self.pub = rospy.Publisher('processed_obstacles', Obstacles, queue_size=10)
		self.sub = rospy.Subscriber('obstacles', Obstacles, self.process_obstacles)

	def process_obstacles(self, obstacles_msg):
		self.processed_obstacles_msg.header = obstacles_msg.header
		self.processed_obstacles_msg.segments = []
		self.processed_obstacles_msg.circles = obstacles_msg.circles

		for segm in obstacles_msg.segments:
			equivalent_circle = CircleObstacle()
			center = Point()
			radius = 0

			pt1 = segm.first_point # Point() type
			pt2 = segm.last_point # Point() type
			# Converting to np arrays for manipulations
			# z is ignored due to 2D map
			pt1_arr = np.array([pt1.x, pt1.y])
			pt2_arr = np.array([pt2.x, pt2.y])

			radius = np.linalg.norm(pt1_arr - pt2_arr)/2

			# In order to find center of eqv_circle, we note that
			# first_pt and last_pt are returned in counter-clockwise direction
			# We'll make use of that to define the circle's center away
			# from the bot rather than on the same side

			mid_pt = (pt1_arr+pt2_arr)/2
			R = np.matrix([[0, -1], [1, 0]])

			# Rotate CCW by 90 degrees
			m = np.dot(R, pt1_arr-mid_pt)
			rotated_vector = np.array([float(m.T[0]), float(m.T[1])])
			center_arr = mid_pt + rotated_vector
			center.x = center_arr[0]
			center.y = center_arr[1]
			center.z = 0

			equivalent_circle.radius = radius
			equivalent_circle.center = center
			self.processed_obstacles_msg.circles.append(equivalent_circle)

		self.pub.publish(self.processed_obstacles_msg)

if __name__ == '__main__':
	try:
		rospy.init_node('obstacle_postprocessor_node')
		obs_proc = Obstacle_Processor()
		while (rospy.get_time()==0):
			pass
		rospy.spin()

	except rospy.ROSInterruptException:
		rospy.loginfo("Node terminated")