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
import rospy

class teleop(object):
	def __init__(self):
		rospy.init_node('teleop')
		self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
		self.rate = rospy.Rate(2)
		self.vel_msg = Twist()
		self.vel_msg.linear.x = 0
		self.vel_msg.angular.z = 0
		self.key = None
		self.settings = termios.tcgetattr(sys.stdin)
		self.speed = .1

	def run(self):
		while not rospy.is_shutdown():
			while self.key != '\x03':
				self.key = self.getKey()
				if self.key == 'w':
					self.vel_msg.linear.x = self.speed
				if self.key == 's':
					self.vel_msg.linear.x = -self.speed
				if self.key == 'a':
					self.vel_msg.angular.z = self.speed*3
				if self.key == 'd':
					self.vel_msg.angular.z = -self.speed*3
				if self.key == ' ':
					self.vel_msg.linear.x = 0
					self.vel_msg.angular.z = 0
				self.pub.publish(self.vel_msg)
				self.rate.sleep()
				print(self.key)




	def getKey(self):
		tty.setraw(sys.stdin.fileno())
		select.select([sys.stdin],[],[],0)
		self.key = sys.stdin.read(1)
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
		return self.key

if __name__ == '__main__':
	node = teleop()
	node.run()