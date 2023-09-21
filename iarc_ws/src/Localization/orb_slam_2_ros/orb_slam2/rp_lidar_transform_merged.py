#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point, Twist
import numpy as np
from math import atan2,tan,sin,cos,sqrt
from nav_msgs.msg import Odometry
from quat2euler import quat2euler
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Range
from sensor_msgs.msg import Imu
import time

class LOCALIZATION:
	def __init__(self):
		rospy.init_node('rp_lidar_transform')
		pub = rospy.Publisher('/uav0/mavros/vision_pose/pose', PoseStamped,queue_size=20)
		rospy.Subscriber('/uav0/mavros/imu/data',Imu, self.callback1,queue_size=1)
		rospy.Subscriber('/hunter_killer_mast/proxy_sensor',Range, self.rp_lidar_transform,queue_size=1)
		time.sleep(3)
		rospy.Subscriber('/uav0/orb_slam2_rgbd/pose',PoseStamped, self.callback,queue_size=1)

	def callback(self, data):
		self.orbx = data.pose.position.z - 0.07
		self.orby = data.pose.position.y 
		self.orbz = self.h_apparent + 0.045
		msg = PoseStamped()
		msg.header.stamp = rospy.Time.now()
		msg.header.frame_id = "map"
		msg.pose.position.x = self.orbx
		msg.pose.position.y = self.orby
		msg.pose.position.z = self.orbz
		self.pub.publish(msg)
		
	def rp_lidar_transform(self, data1):
		self.h_apparent = data1.range
		self.h_apparent = (self.h_apparent+0.045)*cos(self.roll)*cos(self.pitch)

	def callback1(self, data):
		x = data.orientation.x
		y = data.orientation.y
		z = data.orientation.z
		w = data.orientation.w

		self.roll = quat2euler(x,y,z,w)[0]
		self.pitch = quat2euler(x,y,z,w)[1]
		self.yaw = quat2euler(x,y,z,w)[2]


if __name__ == '__main__':
	localization = LOCALIZATION()
# global orbx, orby, orbz
# global x1
# global roll
# global pitch
# global yaw
# global y1
# global r1
# global p1
# global h_apparent	
# global msg

# def main():
# 	# global orbx, orby, orbz
# 	# global roll
# 	# global pitch
# 	# global yaw
# 	# global y1
# 	# global h_actual
# 	# global msg


# 	def callback(data):
# 		global h_apparent
# 		global msg
# 		global orbx, orby, orbz
# 		global roll, pitch, yaw
# #		print(h_apparent)
# 		orbx = data.pose.position.z - 0.07
# 		orby = data.pose.position.y 
# 		orbz = h_apparent + 0.045
# 		msg = PoseStamped()
# #		print("assign values")
# 		msg.header.stamp = rospy.Time.now()
# 		msg.header.frame_id = "map"
# 		msg.pose.position.x = orbx
# 		msg.pose.position.y = orby
# 		msg.pose.position.z = orbz
# 		pub.publish(msg)
# #		print('publish message')
		
# 	def rp_lidar_transform(data1):
# 		global h_apparent
# 		h_apparent=data1.range
# 		h_apparent=(h_apparent+0.045)*cos(roll)*cos(pitch)
# #		print(roll)
# #		print(h_apparent)
# 	def callback1(data):

# 		global roll
# 		global pitch
# 		global yaw


# 		x=data.orientation.x
# 		y=data.orientation.y
# 		z=data.orientation.z
# 		w=data.orientation.w


# 		roll = quat2euler(x,y,z,w)[0]
# 		pitch = quat2euler(x,y,z,w)[1]
# 		yaw = quat2euler(x,y,z,w)[2]

	
# 	rospy.init_node('rp_lidar_transform')
# 	pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped,queue_size=20)
# 	rospy.Subscriber('/mavros/imu/data',Imu,callback=callback1,queue_size=1)
	
# 	rospy.Subscriber('/hunter_killer_mast/proxy_sensor',Range,callback=rp_lidar_transform,queue_size=1)
# 	time.sleep(3)
# 	rospy.Subscriber('/orb_slam2_rgbd/pose',PoseStamped, callback=callback,queue_size=1)
	
	
	

# 	rospy.spin()


# main()
