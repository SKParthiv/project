import rospy
from std_msgs.msg import Float32, Int32
from sensor_msgs.msg import LaserScan
import math

#!/usr/bin/env python


class SonarToMap:
	def __init__(self):
		rospy.init_node('sonar_to_map', anonymous=True)

		self.front_distance = 0.0
		self.back_distance = 0.0

		self.laser_pub = rospy.Publisher('scan', LaserScan, queue_size=10)
		self.motor_left_pub = rospy.Publisher('motor_instructions_left', Int32, queue_size=10)
		self.motor_right_pub = rospy.Publisher('motor_instructions_right', Int32, queue_size=10)

		rospy.Subscriber('front_distance', Float32, self.front_distance_callback)
		rospy.Subscriber('back_distance', Float32, self.back_distance_callback)

		self.rate = rospy.Rate(10)  # 10 Hz

	def front_distance_callback(self, data):
		self.front_distance = data.data

	def back_distance_callback(self, data):
		self.back_distance = data.data

	def publish_laser_scan(self):
		scan = LaserScan()
		scan.header.stamp = rospy.Time.now()
		scan.header.frame_id = 'laser_frame'
		scan.angle_min = -math.pi / 2
		scan.angle_max = math.pi / 2
		scan.angle_increment = math.pi / 180
		scan.time_increment = (1.0 / 40) / 360
		scan.range_min = 0.02
		scan.range_max = 4.0

		ranges = [float('inf')] * 360
		ranges[0] = self.front_distance
		ranges[180] = self.back_distance

		scan.ranges = ranges
		self.laser_pub.publish(scan)

	def spin_360(self):
		speed = 50  # Set a manageable speed
		duration = 3  # Duration to complete a 180-degree spin

		self.motor_left_pub.publish(Int32(speed))
		self.motor_right_pub.publish(Int32(-speed))
		rospy.sleep(duration)

		self.motor_left_pub.publish(Int32(0))
		self.motor_right_pub.publish(Int32(0))

	def run(self):
		while not rospy.is_shutdown():
			self.spin_360()
			self.publish_laser_scan()
			self.rate.sleep()

if __name__ == '__main__':
	try:
		sonar_to_map = SonarToMap()
		sonar_to_map.run()
	except rospy.ROSInterruptException:
		pass