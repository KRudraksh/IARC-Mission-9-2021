#!/usr/bin/env python
import rospy
import time
import numpy as np
# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped, PointStamped
from gazebo_msgs.msg import ModelStates, LinkStates

# import all mavros messages and services
from mavros_msgs.msg import PositionTarget, GlobalPositionTarget
from mavros_msgs.srv import SetMode, CommandBool
from threading import Thread
import thread
from utils.offboard import uav, child, parent
from std_msgs.msg import Int32, Float64
import tf


def callback(data):
	global ref
	ref = data
	# print(ref)

def loc_pose( data):
	global child_loc
	child_loc.x = data.pose.position.x
	child_loc.y = data.pose.position.y
	child_loc.z = data.pose.position.z

def loc_vel( data):
	global child_vel
	child_vel.u = data.twist.linear.x
	child_vel.v = data.twist.linear.y
	child_vel.w = data.twist.linear.z

def handle_pose(msg):
	global p
	n = list(msg.name).index('hunter_killer_mast::communication_module')
	#print(b)
	p = msg.pose[n]
	#print(p)

def main():
	rospy.init_node('trial_node', anonymous=True)
	listener = tf.TransformListener()
	global child_loc
	child_loc = Point()
	child_loc.x = 0
	child_loc.y = 0
	child_loc.z = 0
	pub = rospy.Publisher('/uav0/mavros/setpoint_raw/local', PositionTarget, queue_size = 1)
	pub2 = rospy.Publisher('/uav0/mavros/setpoint_position/local', PoseStamped, queue_size=1)
	rospy.Subscriber('/gazebo/link_states', LinkStates, handle_pose)
	rospy.Subscriber('/uav0/mavros/local_position/pose', PoseStamped, loc_pose)


	#print("1")
	uav1 = child()
	ref = Point()
	child_loc = Point()
	#print("2")
	# rospy.Subscriber("/pubb", Point, callback)	
	uav1.setarmc(1)
	uav1.offboard(1)
	uav1.gotopose(0, 0, 2)
	
	#-------------------------------------------------------------
	#uav1.setoreo(0,0,-0.707,0.707)
	time.sleep(2)
	uav1.gotopose(5.75, 0, 0.75)
	uav1.gotopose(6.75, 0, 0.75)
	uav1.gotopose(7.75, 0, 0.75)
	uav1.gotopose(8.25, 0, 0.75)
	uav1.gotopose(8.75, 0, 0.75)
	uav1.gotopose(9.00, 0, 0.75)
	time.sleep(1)
	uav1.gotopose(9.15, 0, 0.75)
	rate = rospy.Rate(20)
	while not rospy.is_shutdown():
		pt = PoseStamped()
		pt.pose.position.x = p.position.x - 0.4  #mat.point.x - 2
		pt.pose.position.y = p.position.y
		pt.pose.position.z = p.position.z - 0.18
		# print(pt)
		pub2.publish(pt)
		rate.sleep()
		#print('Reached ',x,y,z)
	#---------------------------------------------------------------
	


	#----------------------------------------------------------------
	# uav1.gotopose(8, 0, 0.5)
	# rate = rospy.Rate(20)
	# while not rospy.is_shutdown():
	# 	msg = PositionTarget()
	# 	msg.header.stamp = rospy.Time.now()
	# 	msg.coordinate_frame = 8
	# 	msg.type_mask = 4039
	# 	msg.velocity.x = (p.position.x - uav1.child_loc.x - 1)*0.8
	# 	msg.velocity.y = (p.position.y - uav1.child_loc.y)*0.8
	# 	msg.velocity.z = 0
	# 	r = rospy.Rate(10) # 10hz
	# 	print(msg)
	# 	pub.publish(msg)
	# 	r.sleep()
	#-----------------------------------------------------------------

	# def handle_pose(msg):
	#     n = list(msg.name).index('iris')
	#     #print(b)
	#     p = msg.pose[n]
	#     # st = tf2_ros.StaticTransformBroadcaster()
	#     br = tf.TransformBroadcaster()

	#     # tf2Stamp = TransformStamped()
	#     # tf2Stamp.header.stamp = rospy.Time.now()
	#     # tf2Stamp.header.frame_id = "base_link"
	#     # tf2Stamp.child_frame_id = "r200link"
	#     # tf2Stamp.transform.translation.x = 0.1
	#     # tf2Stamp.transform.translation.y = 0.0
	#     # tf2Stamp.transform.translation.z = 0.0

	#     # quat = tf.transformations.quaternion_from_euler(0.0,0.0,0.0)

	#     # tf2Stamp.transform.rotation.x = quat[0]
	#     # tf2Stamp.transform.rotation.y = quat[1]
	#     # tf2Stamp.transform.rotation.z = quat[2]
	#     # tf2Stamp.transform.rotation.w = quat[3]

	#     # st.sendTransform(tf2Stamp)

	#     br.sendTransform((p.position.x, p.position.y, p.position.z),
	#                      (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w),
	#                      rospy.Time.now(),
	#                      "base_link",
	#                      "/map")
	#     # br.sendTransform((msg.pose.position.x, msg.pose.position.y, msg.pose.position.z),
	#     #                  (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w),
	#     #                  rospy.Time.now(),
	#     #                  "base_link",
	#     #                  "/world")

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
