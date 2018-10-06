#!/usr/bin/env python

from __future__ import print_function
from geometry_msgs.msg import PointStamped, PointStamped
from std_msgs.msg import Header
from neato_node.msg import Bump
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import tty
import select
import sys
import termios
import datetime
import rospy

class teleop(object):
	def __init__(self):
		"initialize teleop"
		rospy.init_node('teleop')
		#publishes velocity from keystroke
		self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
		#used for changing distance and direction for wall following
		self.pub2 = rospy.Publisher('/distance', Float32, queue_size = 10)
		self.pub3 = rospy.Publisher('/direction', Float32, queue_size = 10)
		self.rate = rospy.Rate(20)
		#set up vel_msg as a twist
		self.vel_msg = Twist()
		self.vel_msg.linear.x = 0
		self.vel_msg.angular.z = 0
		self.key = None
		self.settings = termios.tcgetattr(sys.stdin)
		self.speed = .3
		#for wall following
		self.target_distance = .6
		self.direction = 1;

	def run(self):
		"Run teleop"
		while not rospy.is_shutdown():
			while self.key != '\x03':
				self.key = self.getKey()
				##wasd runs robot forward, backward, and turns
				if self.key == 'w':
					self.vel_msg.linear.x = self.speed
					self.pub.publish(self.vel_msg)
				if self.key == 's':
					self.vel_msg.linear.x = -self.speed
					self.pub.publish(self.vel_msg)
				if self.key == 'a':
					self.vel_msg.angular.z = self.speed*3
					self.pub.publish(self.vel_msg)
				if self.key == 'd':
					self.vel_msg.angular.z = -self.speed*3
					self.pub.publish(self.vel_msg)
				##if space is pressed, robot vel and angular vel set to 0
				if self.key == ' ':
					self.vel_msg.linear.x = 0
					self.vel_msg.angular.z = 0
					self.pub.publish(self.vel_msg)
				## J and L used to move robot closer to wall or further away
				if self.key == 'j':
					self.target_distance += .05
					print('distance set to:', self.target_distance)
				if self.key == 'l' and self.target_distance > .3:
					self.target_distance -= .05
					print('distance set to:', self.target_distance)
				##i and k switches robot direction
				if self.key == 'i':
					self.direction = 1;
				if self.key == 'k' and self.target_distance > .3:
					self.direction = -1;
				##publish direction and distance as its own topic
				self.pub2.publish(self.target_distance)
				self.pub3.publish(self.direction)
				self.rate.sleep()


	def getKey(self):
		"getKey function"
		tty.setraw(sys.stdin.fileno())
		select.select([sys.stdin],[],[],0)
		self.key = sys.stdin.read(1)
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
		return self.key

if __name__ == '__main__':
	node = teleop()
	node.run()