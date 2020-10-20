#! /usr/bin/env python
# trunc8 did this
import rospy
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import message_filters
import tf
import math

class Obstacle_Detector:
    def __init__(self):
        self.loop_rate = rospy.Rate(1)
        self.obstacle_msg = Float64MultiArray()
        self.pub = rospy.Publisher('obstacles', Float64MultiArray, queue_size=10)

        self.odom = None
        self.scan = None
        self.odom_sub = message_filters.Subscriber('/odom', Odometry)
        self.scan_sub = message_filters.Subscriber('/scan', LaserScan)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.odom_sub, self.scan_sub], 10, 1)
        self.ts.registerCallback(self.updateObstacles)

    def updateObstacles(self, odom_msg, scan_msg):
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
        yaw = rpy[2] # this is theta in the tuple (x,y,theta)

        laser_ranges = self.scan.ranges
        max_range = self.scan.range_max

        obstacle_list = []
        # print(len(laser_ranges)) #1080
        angle = self.scan.angle_min #-270 deg
        increment = self.scan.angle_increment #0.25 deg
        epsilon = 2 # threshold for discontinuity in scan distance range
        view_radius = 5 # threshold of view circle that we want to consider
        for i in range(1,len(laser_ranges)):
            dist1 = laser_ranges[i]
            dist2 = laser_ranges[i-1]
            if(abs(dist1 - dist2) > epsilon):
            	# For the time being including both dist1 and dist2 as obstacle endpoints
                if dist1 < view_radius:
	                x_obstacle = x+dist1*math.cos(yaw+angle)
	                y_obstacle = y+dist1*math.sin(yaw+angle)
	                obstacle_list.extend([x_obstacle, y_obstacle])

                # dist2
                if dist2 < view_radius:
	                x_obstacle = x+dist2*math.cos(yaw+angle)
	                y_obstacle = y+dist2*math.sin(yaw+angle)
	                obstacle_list.extend([x_obstacle, y_obstacle])
            angle += increment

        self.obstacle_msg.data = obstacle_list

    def start(self):
        rospy.loginfo("Start point")
        while not rospy.is_shutdown():
            rospy.loginfo("Running")
            if self.obstacle_msg is not None:
                self.pub.publish(self.obstacle_msg)
            self.loop_rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('obstacle_detector_node', disable_signals=True)
        obs = Obstacle_Detector()
        while (rospy.get_time()==0):
            pass
        obs.start()

    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated")
