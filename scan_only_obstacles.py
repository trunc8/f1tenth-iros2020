#!/usr/bin/env python

# trunc8 did this

import rospy
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import message_filters
import tf
import math
import cv2
import numpy as np
import rospkg

class Scan_Filter:
	def __init__(self):
		self.sc_filter_msg = LaserScan()
		self.pub = rospy.Publisher('/filtered_scan', LaserScan, queue_size=10)

		self.x_origin_px = 0
		self.y_origin_px = 0
		self.resolution = 0
		self.eroded_map = None
		self.get_eroded_map_attributes()

		self.odom = None
		self.scan = None
		self.odom_sub = message_filters.Subscriber('/odom', Odometry)
		self.scan_sub = message_filters.Subscriber('/scan', LaserScan)
		self.ts = message_filters.ApproximateTimeSynchronizer([self.odom_sub, self.scan_sub], queue_size=10, slop=1)
		# "slop" parameter defines the delay (in seconds) with which messages can be synchronized
		self.ts.registerCallback(self.filter_track_out)

	def get_eroded_map_attributes(self):
		rospack = rospkg.RosPack()
		map_path = rospack.get_path('f1tenth_gym_ros') + '/maps/vegas.png'
		img = cv2.imread(map_path)[:,:,0] # Keep a single channel; RGB channels are identical
		# print(img.shape): (2248, 3000)
		kernel = np.ones((10,10), np.uint8) # Structuring element for erosion
		self.eroded_map = cv2.erode(img, kernel)
		# From the vegas.yaml file:
		#   resolution: 0.050000
		#   origin: [-11.606540, -27.320793, 0.000000]
		# Locating this point on our eroded image
		self.resolution = 0.05 # This is the factor of conversion from meters to pixel coordinates
		self.x_origin_px = 11.60654/self.resolution # 232 px
		y_flipped = 27.320793/self.resolution # 546 px
		self.y_origin_px = self.eroded_map.shape[0] - y_flipped # Since OpenCV flips y-axis
		# In vegas map: shape[0] = 2248px

	def filter_track_out(self, odom_msg, scan_msg):
		if odom_msg is not None and scan_msg is not None:
			# print("Time sync works!")
			self.odom = odom_msg
			self.scan = scan_msg
		else:
			return

		x = self.odom.pose.pose.position.x
		y = self.odom.pose.pose.position.y
		quaternion = (
			self.odom.pose.pose.orientation.x,
			self.odom.pose.pose.orientation.y,
			self.odom.pose.pose.orientation.z,
			self.odom.pose.pose.orientation.w)

		rpy = tf.transformations.euler_from_quaternion(quaternion)
		theta = rpy[2] # this is theta in the tuple (x,y,theta)

		laser_ranges = self.scan.ranges
		max_range = self.scan.range_max

		filtered_ranges = []
		# print(len(laser_ranges)) #1080
		angle = self.scan.angle_min #-270 deg
		increment = self.scan.angle_increment #0.25 deg
		for i in range(1, len(laser_ranges)):
			distance = laser_ranges[i]
			x_hit = x + distance*math.cos(theta + angle) 
			y_hit = y + distance*math.sin(theta + angle)
			# x_hit and y_hit are global coordinates in meters adjusted for yaml origin
			# Obtaining corresponding pixel coordinates
			x_hit_px = self.x_origin_px + x_hit/self.resolution
			y_hit_px = self.y_origin_px - y_hit/self.resolution
			# Reason: OpenCV y and map y increase in opposite directions

			# print(y_hit, x_hit)
			# print(y_hit_px, x_hit_px)
			# print distance, "\t", self.eroded_map[int(y_hit_px), int(x_hit_px)]

			# Note for below: If you want to test using imshow, please change:
			# 	img = cv2.imread(map_path)[:,:,0] ==> img = cv2.imread(map_path)[:,:,:]

			# cv2.namedWindow("Morphed",cv2.WINDOW_NORMAL)
			# cv2.resizeWindow("Morphed", 2248,3000)
			# cv2.circle(self.eroded_map, (int(self.x_origin_px), int(self.y_origin_px)), 10, (0,0,255	), 5)
			# cv2.circle(self.eroded_map, (int(x_hit_px), int(y_hit_px)), 10, (255,255,0), 5)
			# cv2.imshow("Morphed", self.eroded_map)
			# print(self.eroded_map[int(self.y_origin_px), int(self.x_origin_px)]) # We must check in this order: img(y,x)
			# print(self.eroded_map[int(y_hit_px), int(x_hit_px)])
			# cv2.waitKey(0)

			# rospy.sleep(3)
			px_lies_on_track_wall = self.eroded_map[int(y_hit_px), int(x_hit_px)] == 0
			# We must check in this order: img(y,x)
			if px_lies_on_track_wall:
				filtered_ranges.append(float('inf'))
			else:
				filtered_ranges.append(distance)
			angle += increment

		# rospy.sleep(3)
		self.sc_filter_msg = self.scan
		self.sc_filter_msg.ranges = filtered_ranges
		self.pub.publish(self.sc_filter_msg)
		rospy.loginfo("sc_filter published!")

if __name__ == '__main__':
	try:
		rospy.init_node('scan_filter_node', disable_signals=True)
		sc_filter = Scan_Filter()
		while (rospy.get_time()==0):
			pass
		rospy.spin()

	except rospy.ROSInterruptException:
		rospy.loginfo("Node terminated")


# Quick Testing of below code
# rospack = rospkg.RosPack()

# img_path = rospack.get_path('f1tenth_gym_ros') + '/maps/vegas.png'

# img = cv2.imread(img_path)[:,:,:] # Keep a single channel; RGB channels are identical
# kernel = np.ones((5,5), np.uint8) # Structuring element for erosion
# img = cv2.erode(img, kernel)
# # From the vegas.yaml file:
# #   resolution: 0.050000
# #   origin: [-11.606540, -27.320793, 0.000000]
# # Locating this point on our eroded img
# resolution = 0.05 # This converts meters to pixel coordinates
# x_origin = 11.60654/resolution
# y_notional = 27.320793/resolution
# y_origin = 2248 - y_notional # Since OpenCV flips y-axis
# cv2.namedWindow("Morphed",cv2.WINDOW_NORMAL)
# cv2.resizeWindow("Morphed", 2248,3000)
# cv2.circle(img, (int(x_origin), int(y_origin)), 10, (255,255,0), 5)
# cv2.imshow("Morphed", img)
# print(img.shape)
# print(img[int(y_origin), int(x_origin)]) # We must check in this order: img(y,x)
# cv2.waitKey(0)