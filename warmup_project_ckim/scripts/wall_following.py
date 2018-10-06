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
import select
import sys

class wall_following(object):
	"follow wall"

	def __init__(self):
		"initializes wall following"
		rospy.init_node('wall_following')
		self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
		self.pub3 = rospy.Publisher('Robot_marker', Marker, queue_size = 10)
		rospy.Subscriber('/bump', Bump, self.read_bump)
		rospy.Subscriber('/scan', LaserScan, self.read_scan)
		#subscribes to distance and direction node from teleop
		rospy.Subscriber('/distance', Float32, self.read_distance)
		rospy.Subscriber('/direction', Float32, self.read_direction)
		self.rate = rospy.Rate(20)
		self.vel_msg = Twist()
		self.vel_msg.linear.x = 0
		self.vel_msg.angular.z = 0
		#initialize linear and angular vel limit
		self.linear_vel = .2
		self.angular_vel_limit = 1
		self.ranges = None
		self.stop = False
		#these are the angles it checks the wall geometry
		self.scan_angle_1 = 270-30 #in degrees
		self.scan_angle_2 = 270+30 # in degrees
		self.theta = 0
		#line slope and intercept
		self.m = 0
		self.b = 0
		#set up PID constants and variables
		self.kp_angle = .03
		self.kd_angle = .02
		self.derivative_angle = 0
		#delta time for Kd
		self.delta_time = .1
		self.prev_time = 0
		self.current_time = 0
		self.previous_theta = 0
		self.error_theta = 0
		self.target_theta = 0 #in degrees
		self.error_distance = 0
		self.target_distance = .6
		self.closest_distance = .6
		self.kp_distance = 120
		self.theta_limit = 20
		self.direction = 1

	def read_distance(self, data):
		"reads how far from wall"
		self.target_distance = data.data
		print("distance set to:", self.target_distance)

	def read_direction(self,data):
		"reads direction from teleop"
		self.direction = data.data

	def read_bump(self, data):
		"reads bump sensor"
		if data.leftFront or data.leftSide or data.rightFront or data.rightSide:
			self.stop = True
		else:
			self.stop = False

	def read_scan(self, data):
		"reads scan data"
		self.ranges = data.ranges

	def calculate_angle(self):
		"calculates angle from wall"
		x1 = self.ranges[self.scan_angle_1]*math.cos(math.radians(270 - self.scan_angle_1))
		y1 = -self.ranges[self.scan_angle_1]*math.sin(math.radians(270 - self.scan_angle_1))
		x2 = self.ranges[self.scan_angle_2]*math.cos(math.radians(self.scan_angle_2 - 270))
		y2 = self.ranges[self.scan_angle_2]*math.sin(math.radians(self.scan_angle_2 - 270))
		#if we get no readings, dont calculate slope and intercept
		if(x1 != 0 and y1 != 0 and x2 != 0 and y2 != 0 and x1-x2 != 0):
			self.m = (y1-y2)/(x1-x2)
			self.b = y1-self.m*x1
		#if the slope isnt 0
		if(self.m != 0):
			#calculates the angle from parallel to the wall
			self.theta = math.degrees(math.atan(1/self.m))

	def PID_control(self):
		"controls linear and angular motion"
		#calculates the perpendicular distance to wall
		self.closest_distance = abs(self.b)/math.sqrt(1+self.m*self.m)
		#target distance is set by teleop
		self.error_distance = self.target_distance - self.closest_distance
		#sets target theta with proportional control
		#farther from wall, sets target as negative
		#closer to wall, target set as positive
		self.target_theta = self.error_distance*self.kp_distance*self.direction
		#makes sure that target theta is not beyond the theta limit
		if(self.target_theta > self.theta_limit):
			self.target_theta = self.theta_limit
		elif(-self.target_theta > self.theta_limit):
			self.target_theta = -self.theta_limit
		#next control loop for error of robot angle from target
		self.error_theta = self.target_theta - self.theta
		#set up derivative for theta
		self.current_time = rospy.get_time()
		if(self.current_time - self.prev_time > self.delta_time):
			self.derivative_angle = self.kd_angle*(self.previous_theta - self.error_theta)
			self.previous_theta = self.error_theta
			self.prev_time = self.current_time

		#sets angular velocity with PD control
		angular_vel = self.error_theta*self.kp_angle - self.derivative_angle
		#bounds angular vel under limit
		if(angular_vel > self.angular_vel_limit):
			angular_vel = self.angular_vel_limit
		elif(-angular_vel > self.angular_vel_limit):
			angular_vel = -self.angular_vel_limit
		self.vel_msg.angular.z = angular_vel
		#self.direction sets direction
		self.vel_msg.linear.x = self.linear_vel*self.direction
	
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
		
	def run(self):
		while not rospy.is_shutdown():
			#makes sure we do not have empty range of scans
			if(self.ranges is not None):
				self.calculate_angle()
			self.draw_marker()
			self.PID_control()
			self.pub.publish(self.vel_msg)
			self.pub3.publish(self.marker2)
			self.rate.sleep()

if __name__ == '__main__':
	node = wall_following()
	node.run()