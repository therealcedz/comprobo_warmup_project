#!/usr/bin/env python

from __future__ import print_function
from geometry_msgs.msg import PointStamped, PointStamped
from std_msgs.msg import Header
from neato_node.msg import Bump
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
import rospy
import math
import tty
import select
import sys
import termios

class person_following(object):
	"follow a person"

	def __init__(self):
		"initialize person following"
		rospy.init_node('person_following')
		self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
		rospy.Subscriber('/bump', Bump, self.read_bump)
		rospy.Subscriber('/scan', LaserScan, self.read_scan)
		self.pub2 = rospy.Publisher('Person_Marker', Marker, queue_size = 10)
		self.pub3 = rospy.Publisher('Robot_marker', Marker, queue_size = 10)
		self.marker2 = Marker()
		self.marker = Marker()
		self.rate = rospy.Rate(20)
		self.vel_msg = Twist()
		self.vel_msg.linear.x = 0
		self.vel_msg.angular.z = 0
		self.linear_vel = .2
		self.angular_vel_limit = 1
		self.ranges = None
		#scanning range from positive y axis
		self.location_range_max = 45	# degrees
		self.location_range_min = -45	# degrees
		self.prev_range = None
		self.current_range = None
		self.prev_range2 = None
		self.current_range2 = None
		self.range_threshold = .3
		self.object_array = []
		self.object_size_array = []
		self.size_min = 0.01
		self.size_max = 0.04
		self.target_theta = 0
		self.theta = 0
		self.target_distance = 1
		self.error_distance = 0
		self.distance = .5
		self.linear_vel_limit = .2
		self.angular_vel_limit = 1
		self.kp_angle = .02
		self.kd_angle = .02
		self.kp_distance = .5
		self.derivative_angle = 0
		self.delta_time = .1
		self.prev_time = 0
		self.current_time = 0
		self.previous_theta = 0


	def read_scan(self, data):
		"reads lidar data"
		self.ranges = data.ranges

	def read_bump(self, data):
		"reads bump data"
		if data.leftFront or data.leftSide or data.rightFront or data.rightSide:
			self.stop = True
		else:
			self.stop = False

	def locate_person(self):
		"creates array of location of potential legs"
		self.object_array = []
		angle = 0
		self.prev_range = 0
		#loop through each angle in the scanning range
		for angle in range(self.location_range_min, self.location_range_max):
			#modify for negative angles
			if(angle < 1):
				angle = angle + 360
			self.current_range = self.ranges[angle]
			#if we dont scan anything
			if(self.current_range != 0):
				#locate first "jump" in scan
				if (self.prev_range - self.current_range > self.range_threshold):
					angle2 = 0
					current_object_array = []
					#reset angle to negative value 
					if(angle > 45):
						angle = angle - 360
					self.prev_range2 = 0
					#loop through starting from first jump
					for angle2 in range(angle + 1, self.location_range_max):
						#correct for negative values
						if(angle < 1):
							angle = angle + 360
						if(angle2 < 1):
							angle2 = angle2 + 360
						self.current_range2 = self.ranges[angle2]
						#append distances together in array until second jump
						current_object_array.append(self.current_range2)
						#check if second angle is not zero
						if(self.current_range2 != 0):
							#locate second "jump" in scan
							if(self.current_range2 - self.prev_range2 > self.range_threshold):
								#take the average of all distances in object array
								average_range = sum(current_object_array)/len(current_object_array)
								#add jump 1, jump 2 angle, and average range
								self.object_array.append([angle, angle2, average_range])
							self.prev_range2 = self.current_range2
				self.prev_range = self.current_range
		self.object_size_array = []
		#for loop takes length of the objects
		for range_object in self.object_array:
			angle = range_object[0]
			angle2 = range_object[1]
			dist = range_object[2]
			if angle > 45:
				angle = angle - 360
			if angle2 > 45:
				angle2 = angle2 - 360
			#calculates size of objects
			size = math.tan(math.radians(angle2 - angle)*dist)
			#if the size is within the threshold, take the object's angle and distance
			if size < self.size_max and size > self.size_min:
				self.object_size_array.append([(angle2 + angle)/2.0, dist])

	def PID_control(self):
		"PID control loop"
		if(len(self.object_size_array) > 0):
			self.theta = 0
			self.distance = 0
			#take the average of the objects in the object size array
			for thing in self.object_size_array:
				self.theta += thing[0]
				self.distance += thing[1]
			self.theta = self.theta/len(self.object_size_array)
			self.distance = self.distance/len(self.object_size_array)
			#calculate error to legs
			self.error_distance = self.distance - self.target_distance
			#calculate error of theta to legs
			self.error_theta = self.theta - self.target_theta
			#calculate derivative
			self.current_time = rospy.get_time()
			if(self.current_time - self.prev_time > self.delta_time):
				self.derivative_angle = self.kd_angle*(self.previous_theta - self.error_theta)
				self.previous_theta = self.error_theta
				self.prev_time = self.current_time
			#calculate linear and angular velocities
			angular_vel = self.error_theta*self.kp_angle + self.derivative_angle*self.kd_angle
			linear_vel = self.error_distance*self.kp_distance
			#cap linear and angular vel at certain limits
			if(angular_vel > self.angular_vel_limit):
				angular_vel = self.angular_vel_limit
			elif(-angular_vel > self.angular_vel_limit):
				angular_vel = -self.angular_vel_limit
			if(linear_vel > self.linear_vel_limit):
				linear_vel = self.linear_vel_limit
			elif(-linear_vel > self.linear_vel_limit):
				linear_vel = -self.linear_vel_limit
			self.vel_msg.angular.z = angular_vel
			self.vel_msg.linear.x = linear_vel
		else:
			self.vel_msg.angular.z = 0
			self.vel_msg.linear.x = 0

	def run(self):
		"run person_following"
		while not rospy.is_shutdown():
			if(self.ranges is not None):
				self.locate_person()
				self.PID_control()
				self.pub.publish(self.vel_msg)
				self.draw_marker()
				self.pub2.publish(self.marker)
				self.pub3.publish(self.marker2)
				self.rate.sleep()

	def draw_marker(self):
		"draws marker for person and robot"
		self.marker = Marker()
		self.marker.header.frame_id = "base_link"
		self.marker.type = self.marker.CUBE
		self.marker.pose.position.x = math.cos(math.radians(self.theta))*self.distance
		self.marker.pose.position.y = math.sin(math.radians(self.theta))*self.distance
		self.marker.pose.position.z = 0
		self.marker.scale.x = .2
		self.marker.scale.y = .2
		self.marker.scale.z = .2
		self.marker.color.a = 1
		self.marker.color.g= 1
		self.marker2 = Marker()
		self.marker2.header.frame_id = "base_link"
		self.marker2.type = self.marker.SPHERE
		self.marker2.pose.position.x = 0
		self.marker2.pose.position.y = 0
		self.marker2.pose.position.z = 0
		self.marker2.scale.x = .2
		self.marker2.scale.y = .2
		self.marker2.scale.z = .2
		self.marker2.color.a = 1
		self.marker2.color.b = 1

if __name__ == '__main__':
	node = person_following()
	node.run()