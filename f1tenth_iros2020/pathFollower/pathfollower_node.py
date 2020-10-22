#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Oct 21 21:49:37 2020

@author: ckjensen
"""

import rospy, math
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped

def convert_trans_rot_vel_to_steering_angle(v, omega, wheelbase):
  if omega == 0 or v == 0:
    return 0

  radius = v / omega
  return math.atan(wheelbase / radius)


def cmd_callback(data, wheelbase, ackermann_cmd_topic, frame_id, pub):
  v = data.linear.x
  steering = convert_trans_rot_vel_to_steering_angle(v, data.angular.z, wheelbase)

  msg = AckermannDriveStamped()
  msg.header.stamp = rospy.Time.now()
  msg.header.frame_id = frame_id
  msg.drive.steering_angle = steering
  msg.drive.speed = v

  pub.publish(msg)


if __name__ == '__main__':
  try:
    rospy.init_node('cmd_vel_to_ackermann_drive')

    # twist_cmd_topic = rospy.get_param('~twist_cmd_topic', '/cmd_vel')
    # ackermann_cmd_topic = rospy.get_param('~ackermann_cmd_topic', '/ackermann_cmd')
    # wheelbase = rospy.get_param('~wheelbase', 1.0)
    # frame_id = rospy.get_param('~frame_id', 'odom')

    wheelbase = 0.3302
    twist_cmd_topic = '/191747/trajectory'
    frame_id = 'map'
    ackermann_cmd_topic = '/191747/drive'

    def cb(data): return cmd_callback(data, wheelbase, ackermann_cmd_topic, frame_id, pub)
    rospy.Subscriber(twist_cmd_topic, Twist, cb, queue_size=10)
    pub = rospy.Publisher(ackermann_cmd_topic, AckermannDriveStamped, queue_size=10)

    rospy.loginfo("Node 'cmd_vel_to_ackermann_drive' started.\nListening to %s, publishing to %s. Frame id: %s, wheelbase: %f", "/cmd_vel", ackermann_cmd_topic, frame_id, wheelbase)

    rospy.spin()

  except rospy.ROSInterruptException:
    pass