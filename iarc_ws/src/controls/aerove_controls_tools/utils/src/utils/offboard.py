#!/usr/bin/env python
import rospy
import time
import numpy as np
# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped
# import all mavros messages and services
from mavros_msgs.msg import PositionTarget, GlobalPositionTarget, AttitudeTarget
from mavros_msgs.srv import SetMode, CommandBool
from std_msgs.msg import Int32
from tf.transformations import quaternion_from_euler

class uav:
	def __init__(self):
		rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.loc_pose)
		self.pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=1)
		self.pub2 = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size = 10)
		#self.pub3 = rospy.Publisher('/mavros/setpoint_raw/global', GlobalPositionTarget, queue_size = 10)
		self.loc = Point()
		

	def setmode(self,md):
		rospy.wait_for_service('/mavros/set_mode')
		try:
			mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
			response = mode(0,md)
			response.mode_sent
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

	def setarm(self,av): # input: 1=arm, 0=disarm
		rospy.wait_for_service('/mavros/cmd/arming')
		try:
			arming = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
			response = arming(av)
			response.success
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

	def gotopose(self, x, y ,z):
		rate = rospy.Rate(20)
		self.sp = PoseStamped()
		self.sp.pose.position.x = x
		self.sp.pose.position.y = y
		self.sp.pose.position.z = z
		dist = np.sqrt(((self.loc.x-x)**2) + ((self.loc.y-y)**2) + ((self.loc.z-z)**2))
		while(dist > 0.18):
			self.pub.publish(self.sp)
			dist = np.sqrt(((self.loc.x-x)**2) + ((self.loc.y-y)**2) + ((self.loc.z-z)**2))
			rate.sleep()
		#print('Reached ',x,y,z)

	def getvelBody(self, u, v, w):
		msg = PositionTarget()
		msg.header.stamp = rospy.Time.now()
		msg.coordinate_frame = 8
		msg.type_mask = 4039
		msg.velocity.x = u
		msg.velocity.y = v 
		msg.velocity.z = w 
		r = rospy.Rate(10) # 10hz
		while not rospy.is_shutdown():
		   self.pub2.publish(msg)
		   r.sleep()

	def getvelLocal(self, u, v, w):
		msg = PositionTarget()
		msg.header.stamp = rospy.Time.now()
		msg.coordinate_frame = 8
		msg.type_mask = 4039
		msg.velocity.x = u
		msg.velocity.y = v 
		msg.velocity.z = w 
		r = rospy.Rate(20) # 10hz
		while not rospy.is_shutdown():
		   self.pub2.publish(msg)
		   r.sleep()

	def circle(self, x, y, z):
		msg = PositionTarget()
		msg.header.stamp = rospy.Time.now()
		msg.coordinate_frame = 8
		msg.type_mask = 1543
		msg.velocity.x = 0
		msg.velocity.y = -1 
		msg.velocity.z = 0
		msg.acceleration_or_force.x = -0.2
		msg.acceleration_or_force.y = 0
		msg.acceleration_or_force.z = 0
		msg.yaw_rate = -0.2
	   	self.pub2.publish(msg)
	   	
	   	
	def offboard(self, arg):
		rate = rospy.Rate(10)
		sp = PoseStamped()
		sp.pose.position.x = 0.0
		sp.pose.position.y = 0.0
		sp.pose.position.z = 0.0
		for i in range(10):
			self.pub.publish(sp)
			rate.sleep()
		print('We are good to go!!')
		self.setmode("OFFBOARD")

	def loc_pose(self, data):
		self.loc.x = data.pose.position.x
		self.loc.y = data.pose.position.y
		self.loc.z = data.pose.position.z



class child:
	def __init__(self):
		rospy.Subscriber('/uav0/mavros/local_position/pose', PoseStamped, self.loc_pose)
		self.childpub = rospy.Publisher('/uav0/mavros/setpoint_position/local', PoseStamped, queue_size=1)
		self.childpub2 = rospy.Publisher('/uav0/mavros/setpoint_raw/local', PositionTarget, queue_size = 10)
		self.childpub3 = rospy.Publisher('/uav0/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size = 10)
		#self.pub3 = rospy.Publisher('/uav0/mavros/setpoint_raw/global', GlobalPositionTarget, queue_size = 10)
		self.child_loc = Point()


	def setmode(self,md):
		rospy.wait_for_service('/uav0/mavros/set_mode')
		try:
			mode = rospy.ServiceProxy('/uav0/mavros/set_mode', SetMode)
			response = mode(0,md)
			response.mode_sent
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

	def setarmc(self,av): # input: 1=arm, 0=disarm
		print('child done')
		rospy.wait_for_service('/uav0/mavros/cmd/arming')
		try:
			arming = rospy.ServiceProxy('/uav0/mavros/cmd/arming', CommandBool)
			response = arming(av)
			response.success
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

	def gotopose(self, x, y, z):
		rate = rospy.Rate(20)
		self.sp = PoseStamped()
		self.sp.pose.position.x = x
		self.sp.pose.position.y = y
		self.sp.pose.position.z = z
		dist = np.sqrt(((self.child_loc.x-x)**2) + ((self.child_loc.y-y)**2) + ((self.child_loc.z-z)**2))
		while(dist > 0.25):
			self.childpub.publish(self.sp)
			dist = np.sqrt(((self.child_loc.x-x)**2) + ((self.child_loc.y-y)**2) + ((self.child_loc.z-z)**2))
			rate.sleep()
		#print('Reached ',x,y,z)

	def setoreo(self, r,p,y):
		(x,y,z,w) = quaternion_from_euler(r,p,y)
		msg = AttitudeTarget()
		msg.header.stamp = rospy.Time.now()
		# msg.coordinate_frame = 1
		msg.type_mask = 7
		msg.orientation.x = x
		msg.orientation.y = y
		msg.orientation.z = z
		msg.orientation.w = w
		msg.thrust = 0.3
		r = rospy.Rate(10) # 10hz
		for k in range(10):
		   self.childpub3.publish(msg)
		   r.sleep()

	def getvelLocal(self, u, v, w):
		msg = PositionTarget()
		msg.header.stamp = rospy.Time.now()
		msg.coordinate_frame = 1
		msg.type_mask = 4039
		msg.velocity.x = u
		msg.velocity.y = v 
		msg.velocity.z = w 
		r = rospy.Rate(10) # 10hz
		while not rospy.is_shutdown():
		   self.childpub2.publish(msg)
		   r.sleep()

	def circle(self, x, y, z):
		msg = PositionTarget()
		msg.header.stamp = rospy.Time.now()
		msg.coordinate_frame = 8
		msg.type_mask = 1543
		msg.velocity.x = 0
		msg.velocity.y = -1 
		msg.velocity.z = 0
		msg.acceleration_or_force.x = 0.3
		msg.acceleration_or_force.y = 0
		msg.acceleration_or_force.z = 0
		msg.yaw_rate = 0.3
	   	self.childpub2.publish(msg)
	   	
	   	
	def offboard(self, arg):
		rate = rospy.Rate(10)
		sp = PoseStamped()
		sp.pose.position.x = 0.0
		sp.pose.position.y = 0.0
		sp.pose.position.z = 0.0
		for i in range(10):
			self.childpub.publish(sp)
			rate.sleep()
		print('We are good to go!!')
		self.setmode("OFFBOARD")

	def loc_pose(self, data):
		self.child_loc.x = data.pose.position.x
		self.child_loc.y = data.pose.position.y
		self.child_loc.z = data.pose.position.z
		#print("ho gaya")

	

class parent:
	def __init__(self):
		self.parentpub = rospy.Publisher('/uav1/mavros/setpoint_position/local', PoseStamped, queue_size=1)
		self.parentpub2 = rospy.Publisher('/uav1/mavros/setpoint_raw/local', PositionTarget, queue_size = 10)
		#self.pub3 = rospy.Publisher('/uav1/setpoint_raw/global', GlobalPositionTarget, queue_size = 10)
		self.parent_loc = Point()
		self.checkpub = rospy.Publisher('/check_topic', Int32, queue_size=10)
		self.check = Int32()
		self.check = 0
		#self.check = 0
		#self.checkpub.publish(self.check)
		rospy.Subscriber('/uav1/mavros/local_position/pose', PoseStamped, self.loc_pose)
		

	def setmode(self,md):
		rospy.wait_for_service('/uav1/mavros/set_mode')
		try:
			mode = rospy.ServiceProxy('/uav1/mavros/set_mode', SetMode)
			response = mode(0,md)
			response.mode_sent
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

	def setarmp(self,av): # input: 1=arm, 0=disarm
		print('parent done')
		rospy.wait_for_service('/uav1/mavros/cmd/arming')
		try:
			arming = rospy.ServiceProxy('/uav1/mavros/cmd/arming', CommandBool)
			response = arming(av)
			response.success
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

	def gotopose(self, x, y, z):
		rate = rospy.Rate(20)
		self.sp = PoseStamped()
		self.sp.pose.position.x = x
		self.sp.pose.position.y = y
		self.sp.pose.position.z = z
		dist = np.sqrt(((self.parent_loc.x-x)**2) + ((self.parent_loc.y-y)**2) + ((self.parent_loc.z-z)**2))
		self.check = 1.0
		while(dist > 3):
			#print('ab true hona chahiye')
			self.parentpub.publish(self.sp)
			dist = np.sqrt(((self.parent_loc.x-x)**2) + ((self.parent_loc.y-y)**2) + ((self.parent_loc.z-z)**2))
			rate.sleep()
		
		#print('Reached ',x,y,z)

	def gotopose_oreo(self, x, y, z, p=0, q=0, r=0, s=0):
		rate = rospy.Rate(20)
		self.sp = PoseStamped()
		self.sp.pose.position.x = x
		self.sp.pose.position.y = y
		self.sp.pose.position.z = z
		self.sp.pose.orientation.x = p
		self.sp.pose.orientation.y = q
		self.sp.pose.orientation.z = r
		self.sp.pose.orientation.w = s
		dist = np.sqrt(((self.parent_loc.x-x)**2) + ((self.parent_loc.y-y)**2) + ((self.parent_loc.z-z)**2))
		self.check = 1.0
		while(dist > 3):
			self.parentpub.publish(self.sp)
			dist = np.sqrt(((self.parent_loc.x-x)**2) + ((self.parent_loc.y-y)**2) + ((self.parent_loc.z-z)**2))
			rate.sleep()

	def getvelBody(self, u, v, w):
		msg = PositionTarget()
		msg.header.stamp = rospy.Time.now()
		msg.coordinate_frame = 8
		msg.type_mask = 4039
		msg.velocity.x = u
		msg.velocity.y = v 
		msg.velocity.z = w 
		r = rospy.Rate(10) # 10hz
		while not rospy.is_shutdown():
		   self.parentpub2.publish(msg)
		   r.sleep()

	def getvelLocal(self, u, v, w):
		msg = PositionTarget()
		msg.header.stamp = rospy.Time.now()
		msg.coordinate_frame = 1
		msg.type_mask = 4039
		msg.velocity.x = u
		msg.velocity.y = v 
		msg.velocity.z = w 
		r = rospy.Rate(10) # 10hz
		while not rospy.is_shutdown():
		   self.parentpub2.publish(msg)
		   r.sleep()

	def circle(self, x, y, z, r, v):
		self.gotopose(x + 7 + 3, y, z + 5)
		self.gotopose(x + 5, y, z)
		
		rate = rospy.Rate(20) # 20hz
		while not rospy.is_shutdown():
			msg = PositionTarget()
			msg.header.stamp = rospy.Time.now()
			msg.coordinate_frame = 8
			msg.type_mask = 1543
			msg.velocity.x = 0
			msg.velocity.y = -1 
			msg.velocity.z = 0
			msg.acceleration_or_force.x = -0.2
			msg.acceleration_or_force.y = 0
			msg.acceleration_or_force.z = 0
			msg.yaw_rate = -0.2
		   	self.parentpub2.publish(msg)
		   	rate.sleep()

	def offboard(self, arg):
		rate = rospy.Rate(10)
		sp = PoseStamped()
		sp.pose.position.x = 0.0
		sp.pose.position.y = 0.0
		sp.pose.position.z = 0.0
		for i in range(10):
			self.parentpub.publish(sp)
			rate.sleep()
		print('We are good to go!!')

		self.setmode("OFFBOARD")

	def og_gotopose(self,x,y,z,v):
		#rate=rospy.Rate(50)
		og=PositionTarget()
		og.coordinate_frame=1
		og.type_mask=1+2+4+64+128+256+512+1024+2048
		v_vector=np.array([x-self.parent_loc.x,y-self.parent_loc.y,z-self.parent_loc.z])
		unit_vector=v_vector/np.linalg.norm(v_vector)
		velo=unit_vector*v
		og.velocity.x=velo[0]
		og.velocity.y=velo[1]
		og.velocity.z=velo[2]

		self.parentpub2.publish(og)

	def loc_pose(self,data):
		self.parent_loc.x = data.pose.position.x
		self.parent_loc.y = data.pose.position.y
		self.parent_loc.z = data.pose.position.z
		#print('publish hori values')
		flag = self.check 
		#print(flag)
		self.checkpub.publish(flag)

# if __name__ == '__main__':
# 	rospy.init_node('offboard_node', anonymous=True)
# 	rospy.Subscriber('mavros/local_position/pose', PoseStamped, loc_pose)
# 	pub = rospy.Publisher('/uav1/setpoint_position/local', PoseStamped, queue_size=1)
# 	setarm(1)
# 	time.sleep(2)
# 	setmode('OFFBOARD')
# 	gotopose(3,3,3)
# 	gotopose(-5,-5,5)