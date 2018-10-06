#!/usr/bin/env python

from __future__ import print_function
from geometry_msgs.msg import PointStamped, PointStamped
from std_msgs.msg import Header
from neato_node.msg import Bump
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from operator import add
import rospy
import math
import tty
import select
import sys
import termios

class obstacle_avoidance(object):
	"avoid obstacles"

	def __init__(self):
		"initializes obstacle avoidance"
		rospy.init_node('obstacle_avoidance')
		self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
		self.pub3 = rospy.Publisher('Robot_marker', Marker, queue_size = 10)
		rospy.Subscriber('/bump', Bump, self.read_bump)
		rospy.Subscriber('/scan', LaserScan, self.read_scan)
		self.rate = rospy.Rate(20)
		self.vel_msg = Twist()
		self.vel_msg.linear.x = 0
		self.vel_msg.angular.z = 0
		self.linear_vel = .2
		self.angular_vel_limit = 1
		self.ranges = None
		self.location_range_max = 45	# degrees
		self.location_range_min = -45	# degrees
		self.stop = False
		#splits up field of vision to .2m
		self.degree_spacing = 10
		self.sorted_ranges = None
		self.dist_array = None
		self.max_pos = None
		self.arbiter = [0,1,2,3,4,3,2,1,0]
		self.arbiter = [i*.5 for i in self.arbiter]
		self.heading = 0
		self.error_theta = 0
		self.target_distance = 1
		self.error_distance = 0
		self.distance = .5
		self.linear_vel_limit = .2
		self.angular_vel_limit = 1
		self.kp_angle = .05
		self.kd_angle = .02
		self.kp_distance = .5
		self.derivative_angle = 0
		self.delta_time = .1
		self.prev_time = 0
		self.current_time = 0
		self.previous_theta = 0
		self.min_dist = 1

	def return_sorted_ranges(self):
		"sorts ranges by location_range_min to location_range_max"
		if(self.ranges != None):
			self.sorted_ranges = []
			for i in range(self.location_range_max, 0, -1):
				self.sorted_ranges.append(self.ranges[i])
			range_min_adjusted = self.location_range_min + 360
			for i in range(range_min_adjusted-1, 360):
				self.sorted_ranges.append(self.ranges[i])

	def return_heading(self):
		"returns wanted heading of robot"
		current_angle = 0
		self.dist_array = []
		#sorted_ranges - leftside = start, right = end
		sum_dist = 0
		if(self.sorted_ranges != None):
			#for each angle in the scan
			for angle in range(0, len(self.sorted_ranges)):
				dist = self.sorted_ranges[angle]
				#if dist = 0 (far or very close), set the distance to 2.
				if(dist == 0):
					dist = 2
				#sum up the distances
				sum_dist += dist
				#until we reach the degree spacing
				if(angle%self.degree_spacing == 0):
					#takes average of the sums of distances for each spacing
					self.dist_array.append(sum_dist/self.degree_spacing)
					sum_dist = 0
					current_angle += self.degree_spacing
			#remove beginning garbage element
			self.dist_array.pop(0)
			if(len(self.dist_array) != 0):
				#add together the distance array and arbiter
				self.dist_array= list(map(add, self.dist_array, self.arbiter))
				#gets the minimum index and value
				dist_array_min = 500
				min_dist_index = 5
				for i in range(0, len(self.dist_array)):
					if dist_array_min > self.dist_array[i] and self.dist_array[i] != 0:
						min_dist_index = i
						dist_array_min = self.dist_array[i]
				#takes the minimum value, and adds distance to opposite side
				self.dist_array[8-min_dist_index] += (5 - dist_array_min)*.5
				#takes the maximum distance located
				self.max_pos = self.dist_array.index(max(self.dist_array))
				#and turns it into an angle 
				self.heading = self.location_range_max - self.max_pos*(self.degree_spacing) - self.degree_spacing/2.0
			#takes minimum distance for pid linear velocity control
			dist_minimum = 500
			for dist in self.sorted_ranges:
				if dist < dist_minimum and dist != 0:
					self.min_dist = dist
	
	def draw_marker(self):
		"draws robot marker"
		self.marker2 = Marker()
		self.marker2.header.frame_id = "base_link"
		self.marker2.type = self.marker2.SPHERE
		self.marker2.pose.position.x = 0
		self.marker2.pose.position.y = 0
		self.marker2.pose.position.z = 0
		self.marker2.scale.x = .2
		self.marker2.scale.y = .2
		self.marker2.scale.z = .2
		self.marker2.color.a = 1
		self.marker2.color.b = 1

	def PID_control(self):
		#calculates angular error and linear error
		self.error_theta = self.heading
		self.error_distance = self.min_dist - self.target_distance
		#calculates derivative for angular
		self.current_time = rospy.get_time()
		if(self.current_time - self.prev_time > self.delta_time):
			self.derivative_angle = self.kd_angle*(self.previous_theta - self.error_theta)
			self.previous_theta = self.error_theta
			self.prev_time = self.current_time
		#sets angular velocity and linear velocity
		angular_vel = self.error_theta*self.kp_angle + self.derivative_angle*self.kd_angle
		linear_vel = self.error_distance*self.kp_distance
		#caps angular and linear velocities to limits
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
		#if bump sensor is pressed
		if(self.stop):
			self.vel_msg.angular.z = 0
			self.vel_msg.linear.x = 0

	def read_scan(self, data):
		"reads laser scan data"
		self.ranges = data.ranges


	def read_bump(self, data):
		"reads bump sensor"
		if data.leftFront or data.leftSide or data.rightFront or data.rightSide:
			self.stop = True
		else:
			self.stop = False

	def run(self):
		"runs obstacle_avoidance"
		while not rospy.is_shutdown():
			self.draw_marker()
			self.return_sorted_ranges()
			self.return_heading()
			self.PID_control()
			self.pub.publish(self.vel_msg)
			self.pub3.publish(self.marker2)
			self.rate.sleep()



if __name__ == '__main__':
	node = obstacle_avoidance()
	node.run()