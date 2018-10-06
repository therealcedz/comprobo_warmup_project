#!/usr/bin/env python

from __future__ import print_function
from geometry_msgs.msg import PointStamped, PointStamped
from std_msgs.msg import Header
from neato_node.msg import Bump
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import tty
import select
import sys
import termios
import datetime
import time
import rospy

class square(object):
	"drives neato in a square"

	def __init__(self):
		"initialize drive square"
		rospy.init_node('drive_square')
		self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
		rospy.Subscriber('/bump', Bump, self.read_bump)
		self.rate = rospy.Rate(10)
		self.vel_msg = Twist()
		self.vel_msg.linear.x = 0
		self.vel_msg.angular.z = 0
		self.angular_time = 1.65
		self.linear_time = 4
		self.linear_vel = .3
		self.angular_vel = 1
		self.start = time.time()
		self.end = time.time()
		self.stop = False

	def run(self):
		"run square"
		while not rospy.is_shutdown():
			#function uses a timer to see what stage the robot is in
			if(self.end - self.start < self.linear_time): #first leg of square
				self.vel_msg.linear.x = self.linear_vel
			elif(self.end - self.start < self.linear_time + self.angular_time):#first 90 turn
				self.vel_msg.linear.x = 0
				self.vel_msg.angular.z = -self.angular_vel
			elif(self.end - self.start < self.linear_time*2 + self.angular_time):#second leg
				self.vel_msg.linear.x = self.linear_vel
				self.vel_msg.angular.z = 0
			elif(self.end - self.start < self.linear_time*2 + self.angular_time*2):# second 90 turn
				self.vel_msg.linear.x = 0
				self.vel_msg.angular.z = -self.angular_vel
			elif(self.end - self.start < self.linear_time*3 + self.angular_time*2):#third leg
				self.vel_msg.linear.x = self.linear_vel
				self.vel_msg.angular.z = 0
			elif(self.end - self.start < self.linear_time*3 + self.angular_time*3):# third 90 turn
				self.vel_msg.linear.x = 0
				self.vel_msg.angular.z = -self.angular_vel
			elif(self.end - self.start < self.linear_time*4 + self.angular_time*3):#last leg
				self.vel_msg.linear.x = self.linear_vel
				self.vel_msg.angular.z = 0
			elif(self.end - self.start < self.linear_time*4 + self.angular_time*4):# last 90 turn
				self.vel_msg.linear.x = 0
				self.vel_msg.angular.z = -self.angular_vel
			else:
				self.vel_msg.linear.x = 0
				self.vel_msg.angular.z = 0
			#resets velocity to 0 if the bump sensor is pressed
			if self.stop:
				self.vel_msg.linear.x = 0
				self.vel_msg.angular.z = 0
			self.pub.publish(self.vel_msg)
			self.rate.sleep()
			self.end = time.time()

	def read_bump(self, data):
		"read if the robot bumps into things"
		#if any bump sensor is pressed, we stop the robot
		if data.leftFront or data.leftSide or data.rightFront or data.rightSide:
			self.stop = True
			#stop is never set to false, so we have to restart the node




if __name__ == '__main__':
	node = square()
	node.run()