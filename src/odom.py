#!/usr/bin/env python

import rospy
import tf
import math
from geometry_msgs.msg import Vector3

# parameters for computing odometry
pose_x = 0
pose_y = 0
yaw = 0
car_pose_br = tf.TransformBroadcaster()

rospy.init_node('odom_node')

def callback(msg):
	global pose_x, pose_y, yaw

	pose_x = msg.x
	pose_y = msg.y
	yaw = msg.z

	car_pose_br.sendTransform((pose_x, pose_y, 0),
								tf.transformations.quaternion_from_euler(0, 0, yaw),
								rospy.Time.now(),
								"car_base_link",
								"world")

position_sub = rospy.Subscriber('/position', Vector3, callback)

if __name__ == '__main__':
	rate = rospy.Rate(20)
	while not rospy.is_shutdown():
		rate.sleep()
