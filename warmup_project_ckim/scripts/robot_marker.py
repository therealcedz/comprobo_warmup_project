#!/usr/bin/env python

from __future__ import print_function
from geometry_msgs.msg import PointStamped, PointStamped
from std_msgs.msg import Header
from neato_node.msg import Bump
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
import tty
import select
import sys
import termios
import datetime
import time
import rospy

class robot_marker(object):
	"add a marker. Testing if marker works"

	def __init__(self):
		rospy.init_node('robot_marker')
		self.pub = rospy.Publisher('visualization_marker', Marker, queue_size = 10)
		self.rate = rospy.Rate(10)
		self.marker = Marker()

	def draw_marker(self):
		self.marker = Marker()
		self.marker.header.frame_id = "base_link"
		self.marker.type = self.marker.SPHERE
		self.marker.pose.position.x = 0
		self.marker.pose.position.y = 0
		self.marker.pose.position.z = 0
		self.marker.scale.x = .5
		self.marker.scale.y = .5
		self.marker.scale.z = .5
		self.marker.color.a = 1
		self.marker.color.g= 1

	def run(self):
		while not rospy.is_shutdown():
			self.draw_marker()
			self.pub.publish(self.marker)
			self.rate.sleep()

if __name__ == '__main__':
	node = robot_marker()
	node.run()