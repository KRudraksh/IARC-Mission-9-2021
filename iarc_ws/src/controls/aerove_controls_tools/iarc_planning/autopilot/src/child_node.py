#!/usr/bin/env python
import rospy
import rospkg
import time
from time import sleep
from time import time as tm
from quat2euler import quat2euler
import numpy as np
# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Twist, Point, PoseStamped
# import all mavros messages and services
from mavros_msgs.msg import PositionTarget, GlobalPositionTarget, State
from mavros_msgs.srv import SetMode, CommandBool
from std_msgs.msg import Int32, Float64 , String
from nav_msgs.msg import Odometry
from utils.offboard import child, parent
from sensor_msgs.msg import JointState, Range, Imu
from geometry_msgs.msg import Twist

# from gazebo_msgs.msg import ModelState, LinkStates 
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from cv_bridge import CvBridge
import cv2
import ros_numpy
import math
from pyrebase import pyrebase
import tf2_ros
import tf

sys.path.insert(1, '/iarc_ml/pytorch-yolov4/')	# Insert absolute Path in your system to be used.
from tool.darknet2pytorch import Darknet
from demo import *

class PID:
        
    def __init__(self, kp, ki, kd, outMin, outMax, iMin, iMax):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.outMin = outMin
        self.outMax = outMax
        self.intMin = iMin
        self.intMax = iMax

        self.lastError = 0
        self.sumError = 0
        self.lastTime = 0

    def resetValues(self):
        self.lastError = 0
        self.sumError = 0
        self.lastTime = 0

    def pidExecute(self, should, actValue):
        now = rospy.Time.now().to_nsec()
        timeChange = now - self.lastTime
        error = should - actValue
        newErrorSum = self.sumError + (error * timeChange)

        if((newErrorSum >= self.intMin) and (newErrorSum <= self.intMax)):
            self.sumError = newErrorSum
        dError = (error - self.lastError) / timeChange
        output = (self.kp * error) + (self.ki * self.sumError) + (self.kd * dError)
        self.lastError = error
        self.last = now
        if(output > self.outMax):
            output = self.outMax
        if(output < self.outMin):
            output = self.outMin
        return output

class THRESHOLD:
	def __init__(self):
		self.cord = [0,0]
		self.datax_pub = rospy.Publisher("/daughter6/x_position_controller/command", Float64, queue_size = 10)
		self.datay_pub = rospy.Publisher("/daughter6/y_position_controller/command", Float64, queue_size = 10)
		self.dataz_pub = rospy.Publisher("/daughter6/z_position_controller/command", Float64, queue_size = 10)
		rospy.Subscriber("/daughter6/joint_states", JointState, self.state_callback)
		rospy.Subscriber("/camera/depth/points", PointCloud2, self.depth_callback)
		rospy.Subscriber('/comm_module_coord', Point, self.handle_pose)
		self.checkval = 0

	def handle_pose(self, val):
		self.x1 = val.x
		self.y1 = val.y
		self.z1 = val.z

	def state_callback(self, state):
		pos = state.position
		self.x_state = pos[9]
		self.y_state = pos[10]
		self.z_state = pos[0]
		# print(self.checkval)

	def depth_callback(self, data):
		if (self.checkval!= 1):	
			pid_y = PID(0.50, 0.00, 0.10, -0.075, 0.075, -0.001, 0.001)
			output_y = pid_y.pidExecute(0 ,self.x1)
			print("output_y = %f error = %f" %(output_y , 0 - self.x1))
			self.datay_pub.publish(self.y_state + output_y)

			pid_z = PID(0.50, 0.00, 0.10, -0.075, 0.075, -0.001, 0.001)
			output_z = pid_z.pidExecute(0 ,self.y1)
			print("output_z = %f error = %f" %(output_z , 0 - self.y1))
			self.dataz_pub.publish(self.z_state + output_z + 0.1 )

			kp = 0.20
			kd = 0.05
			ki = 0.0
			outMin = -0.075
			outMax = 0.075
			iMin = -0.01
			iMax = 0.01
			
			pid = PID(kp, ki, kd, outMin, outMax, iMin, iMax)
			output = pid.pidExecute(0.157,z1)
			print("True distance = %f ; depth error = %f ; joint_value = %f ; value_published = %f" %(z1 ,(0.157-z1), x_state, (x_state - output)))
			self.datax_pub.publish(self.x_state - output)

class daughter_class:
	def __init__(self):
		self.check = Int32()
		self.child_loc = Point()
		rospy.Subscriber('/uav0/mavros/local_position/pose', PoseStamped, self.loc_pose)
		self.pub2 = rospy.Publisher('/uav0/mavros/setpoint_position/local', PoseStamped, queue_size=1)
		rospy.Subscriber('/uav0/mavros/state', State, self.handle_state)
		config = {
		  "apiKey": "AIzaSyAiBNZfad7r3YewpcqYjc4ko1u7fd2s9yE",
		  "authDomain": "iarc-b9c0a.firebaseapp.com",
		  "databaseURL": "https://iarc-b9c0a-default-rtdb.firebaseio.com/",
		  "storageBucket": "iarc-b9c0a.appspot.com"
		}
		firebase = pyrebase.initialize_app(config)
		storage = firebase.storage()
		self.database = firebase.database()

	def display_text(self,message):
		self.database.child("/display_text").set(message)

		# rospy.Subscriber('/comm_module_coord', Point, self.handle_pose)  UNCOMMENT FOR ML
		# rospy.Subscriber('/gazebo/link_states', LinkStates, self.handle_pose)
	def handle_state(self, msg):
		self.armed = msg.armed


	def handle_pose(self,msg):
		# n = list(msg.name).index('hunter_killer_mast::communication_module')
		# self.p = msg.pose[n]
		pt = PoseStamped()
		pt.pose.position.x = msg.x - 0.5
		pt.pose.position.y = msg.y
		pt.pose.position.z = msg.z - 0.12
		self.pub2.publish(pt)

	def check_func(self, data):
		self.check = data

	def loc_pose(self, data):
		self.child_loc.x = data.pose.position.x
		self.child_loc.y = data.pose.position.y
		self.child_loc.z = data.pose.position.z

class replacement(THRESHOLD):
	def __init__(self, threshold):
		rospy.Subscriber("/daughter6/joint_states", JointState, self.callback)
		# rospy.Subscriber("/gazebo/link_states", LinkStates, self.module_callback)
		rospy.sleep(4)
		
		self.pub_sl = rospy.Publisher('/daughter6/slider_position_controller/command', Float64, queue_size=10)
		self.pub_mg = rospy.Publisher('/daughter6/mast_gripper_position_controller/command', Float64, queue_size=10)
		self.pub_lg = rospy.Publisher('/daughter6/left_gripper_position_controller/command', Float64, queue_size=10)
		self.pub_rg = rospy.Publisher('/daughter6/right_gripper_position_controller/command', Float64, queue_size=10)

		self.pub_tele = rospy.Publisher('/daughter6/telescope_position_controller/command', Float64, queue_size=10)
		self.pub_lan = rospy.Publisher('/daughter6/lantenna_position_controller/command', Float64, queue_size=10)
		self.pub_ran = rospy.Publisher('/daughter6/rantenna_position_controller/command', Float64, queue_size=10)
		self.pub_serv = rospy.Publisher('/daughter6/servo_position_controller/command', Float64, queue_size=10)
		self.pub_z = rospy.Publisher('/daughter6/z_position_controller/command', Float64, queue_size=10)
		rospy.Subscriber('/comm_module_coord', Point, self.handle_pose)

		self.moduleReplaced = 0

		print("==========================================")
		print("REPLACEMENT INITIALIZED")
		print("==========================================")

	def start(self, threshold):
		self.removal(threshold)
		rospy.sleep(0.2)
		self.insertion()

	def handle_pose(self,msg):
		# n = list(msg.name).index('hunter_killer_mast::communication_module')
		# self.p = msg.pose[n]
		self.x_loc_module = msg.x
		self.y_loc_module = msg.y
		self.z_loc_module = msg.z

	# def module_callback(self, data):			# COMMENT OUT FOR ML
	# 	names = data.name
	# 	module_index = names.index("hunter_killer_mast::communication_module")
	# 	self.z_loc_module = data.pose[module_index].position.z
		
	def callback(self,jointstate):	
		self.length = len(jointstate.name)
		for i in range (0,self.length):
			if jointstate.name[i]=='left_antenna_plate_joint':
				self.lan_pos = jointstate.position[i]
			elif jointstate.name[i]=='right_antenna_plate_joint':
				self.ran_pos = jointstate.position[i]
			elif jointstate.name[i]=='mast_gripper_body_joint':
				self.mg_pos = jointstate.position[i]
			elif jointstate.name[i]=='mast_gripper_left_arm_joint':
				self.lg_pos = jointstate.position[i]
			elif jointstate.name[i]=='mast_gripper_right_arm_joint':
				self.rg_pos = jointstate.position[i]
			elif jointstate.name[i]=='servo_joint':
				self.serv_pos = jointstate.position[i]
			elif jointstate.name[i]=='telescope_joint':
				self.tele_pos = jointstate.position[i]
			elif jointstate.name[i]=='slider_joint':
				self.sl_pos = jointstate.position[i]
			elif jointstate.name[i]=='end_effector_joint':
				self.z_pos = jointstate.position[i]
			else:
				pass

	def removal(self, threshold):
		while(self.sl_pos>-0.07):
			self.pub_sl.publish(-0.15)
		rospy.sleep(0.5)

		while(self.mg_pos<0.165):
			self.pub_mg.publish(0.17)
		rospy.sleep(0.5)

		while(self.lg_pos<0.045 and self.rg_pos<0.045):
			self.pub_lg.publish(0.055)
			self.pub_rg.publish(0.055)

		threshold.checkval = 1
		rospy.set_param('/daughter6/x_position_controller/pid/p', 0)
		rospy.set_param('/daughter6/x_position_controller/pid/i', 0)
		rospy.set_param('/daughter6/x_position_controller/pid/d', 0)
		rospy.set_param('/daughter6/y_position_controller/pid/p', 0)
		rospy.set_param('/daughter6/y_position_controller/pid/i', 0)
		rospy.set_param('/daughter6/y_position_controller/pid/d', 0)

		rospy.sleep(2)

		while(self.sl_pos<-0.01):
			self.pub_sl.publish(0.01)
		rospy.sleep(1)

		while(self.lan_pos>-0.085 and self.ran_pos>-0.085):
			self.pub_lan.publish(-0.1)
			self.pub_ran.publish(-0.1)
		rospy.sleep(2)

		while(self.sl_pos>-0.09):
			self.pub_sl.publish(-0.13)

		while(self.lan_pos<-0.01 and self.ran_pos<-0.01):
			self.pub_lan.publish(0)
			self.pub_ran.publish(0)
		rospy.sleep(2)

		while(self.z_loc_module>0.6):
			self.repeat()
		# The gripper should be attached to the rod supporter as of now
		# Communication Module should have been dropped

	def repeat(self):
		while(self.sl_pos<-0.01):
			self.pub_sl.publish(0.01)
		rospy.sleep(1)

		while(self.lan_pos>-0.085 and self.ran_pos>-0.085):
			self.pub_lan.publish(-0.1)
			self.pub_ran.publish(-0.1)
		rospy.sleep(2)

		while(self.sl_pos>-0.09):
			self.pub_sl.publish(-0.13)

		while(self.lan_pos<-0.01 and self.ran_pos<-0.01):
			self.pub_lan.publish(0)
			self.pub_ran.publish(0)
		rospy.sleep(2)

	def insertion(self):
		while(self.mg_pos<0.19):
			self.pub_mg.publish(0.2)

		while(self.serv_pos<1.55):
		 	self.pub_serv.publish(1)

		z_ref = self.z_pos
		rospy.sleep(2)
		while(self.tele_pos>-0.17):
			self.pub_tele.publish(-0.22)
			print('telescope_front')
			#self.pub_z.publish(z_ref + 0.02*math.sin(0.5*float(rospy.get_time())))
		rospy.sleep(1)
		print("back manure started")
		while(self.tele_pos<-0.1):
			self.pub_tele.publish(0)

		#Detachment - Following must be done at this time

		while(self.lg_pos>0.005 and self.rg_pos>0.005):
			self.pub_lg.publish(0)
			self.pub_rg.publish(0)
		rospy.sleep(2)
		while(self.mg_pos>0.005):
			self.pub_mg.publish(0)

class NAVLIGHTS:
	def __init__(self):
		self.listener = tf2_ros.TransformListener()
		rospy.Subscriber("/camera/color/image_raw",Image, self.callback_opencv)
		rospy.Subscriber("/camera/depth/points", PointCloud2, self.callback_depth)
		self.pub2 = rospy.Publisher('/uav0/mavros/setpoint_position/local', PoseStamped, queue_size=1)


	def find_nav_lights(self,cv_img):

		img=cv_img.copy()
		img_hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
		img_gray=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

		b,g,r=np.rollaxis(img,2,0)
		h,s,v=np.rollaxis(img_hsv,2,0)

		mask_gray=((r-g)^2+(g-b)^2+(b-r)^2)>200
		mask_gray=mask_gray*(200<s)
		mask_green = (h<65)*(55<h)*mask_gray 
		#print(mask.shape)
		mask_red = (h<5)+(175<h)
		mask_red=mask_red*mask_gray

		res_green=img_gray*mask_green
		res_red=img_gray*mask_red

		res_green=cv2.GaussianBlur(res_green,(5,5),0)
		res_red=cv2.GaussianBlur(res_red,(5,5),0)

		_,thresh = cv2.threshold(res_red,0,255,cv2.THRESH_BINARY)
		_, contours, hierarchy= cv2.findContours(thresh,cv2.RETR_TREE,
		cv2.CHAIN_APPROX_SIMPLE)
		red_state='Not Found'
		red_centre=(0,0)
		for  cnt in contours :
			# if cv2.contourArea(cnt)<200:
				# continue
			mask=np.zeros_like(res_red)
			mask=cv2.drawContours(mask,[cnt],0,255,-1)
			mask=mask/255.0
			#print(np.sum(img_gray*mask),np.sum(mask))
			M=cv2.moments(cnt)
			red_centre=(M['m10']/M['m00'],M['m01']/M['m00'])
				
			if np.sum(img_gray*mask)/np.sum(mask)>40 :
				red_state='On'
				img=cv2.drawContours(img,[cnt],0,(0,255,0),3)
			else :
				red_state='Off'
				img=cv2.drawContours(img,[cnt],0,(255,255,255),3)


		_,thresh = cv2.threshold(res_green,0,255,cv2.THRESH_BINARY)
		_, contours, hierarchy= cv2.findContours(thresh,cv2.RETR_TREE,
		cv2.CHAIN_APPROX_SIMPLE)
		green_state='Not Found'
		green_centre=(0,0)
		for  cnt in contours :
			# if cv2.contourArea(cnt)<200:
				# continue
			mask=np.zeros_like(res_red)
			mask=cv2.drawContours(mask,[cnt],0,255,-1)
			mask=mask/255.0
			M=cv2.moments(cnt)
			green_centre=(M['m10']/M['m00'],M['m01']/M['m00'])
				
			if np.sum(img_gray*mask)/np.sum(mask)>40 :
				green_state="On"	
				img=cv2.drawContours(img,[cnt],0,(0,0,255),3)
			else :
				green_state="Off"
				img=cv2.drawContours(img,[cnt],0,(128,128,0),3)

		# print(cv_img[rc[1]][rc[0]],cv_img[gc[1]][gc[0]])
		cv2.imshow('img_new',img)
		cv2.waitKey(50)
		return [red_state,np.rint(np.array(red_centre)).astype(np.int32),green_state,np.rint(np.array(green_centre)).astype(np.int32)]
		#res_green=cv2.GaussianBlur(res_green,(5,5),0)
		#res=cv2.inRange(img_hsv,(30,0,0),(90,255,255))
		
		# cv2.imshow('img',cv_img)
		#cv2.imshow('res_green',res_green)
		#cv2.imshow('res_red',res_red)

	def callback_depth(self,value):
		self.pc_arr = ros_numpy.numpify(value)
			

	def callback_opencv(self,data):
		bridge = CvBridge()
		cv_img = bridge.imgmsg_to_cv2(data, "bgr8")
		
		rs,rc,gs,gc = self.find_nav_lights(cv_img)
		wr,hr = rc
		wg,hg = gc
		print(cv_img[rc[1]][rc[0]],cv_img[gc[1]][gc[0]])
		# xp,yp,zp = arr['x'][h][w],arr['y'][h][w],arr['z'][h][w]
		#cord = np.array([xp,yp,zp])
		arr = self.pc_arr
		xpr,ypr,zpr = arr['x'][hr][wr],arr['y'][hr][wr],arr['z'][hr][wr]
		xpg,ypg,zpg = arr['x'][hg][wg],arr['y'][hg][wg],arr['z'][hg][wg]
		psr = PointStamped()
		psr.header.frame_id = "r200link"
		psr.header.stamp = rospy.Time(0)
		psr.point.x = zpr
		psr.point.y = -xpr
		psr.point.z = -ypr
		red_cord = self.listener.transformPoint("/map", psr)
		psg = PointStamped()
		psg.header.frame_id = "r200link"
		psg.header.stamp = rospy.Time(0)
		psg.point.x = zpg
		psg.point.y = -xpg
		psg.point.z = -ypg
		green_cord = self.listener.transformPoint("/map", psg)
		x1 = red_cord.point.x
		y1 = red_cord.point.y
		z1 = red_cord.point.z
		x2 = green_cord.point.x
		y2 = green_cord.point.y
		z2 = green_cord.point.z
		xc,yc,zc = (x1+x2)/2,(y1+y2)/2,(z1+z2)/2

		theta = math.atan((y1-y2)/(x1-x2))
		yaw_to_quat = [math.cos(theta/2), 0, 0, math.sin(theta/2)]
		
		for i in range(20):	
			self.p = PoseStamped()
			self.p.pose.position.x = xc + 0.4*math.sin(theta)
			self.p.pose.position.y = yc - 0.4*math.cos(theta)
			self.p.pose.position.z = zc - 0.48
			self.p.pose.orientation.x = yaw_to_quat[0]
			self.p.pose.orientation.y = yaw_to_quat[1]
			self.p.pose.orientation.z = yaw_to_quat[2]
			self.p.pose.orientation.w = yaw_to_quat[3]
			self.pub2.publish(pt)
		# print(rs,rc,gs,gc)

class MODULEDETECTION:
	def __init__(self):
		print("Module Detection Initiated")
		self.listener = tf2_ros.TransformListener()
		self.board=Darknet('cfg/yolov4_box.cfg',inference=True)
		self.board.load_weights('backup/yolov4_box_4000.weights')
		self.board.cuda()

		self.module=Darknet('cfg/yolov4_module.cfg',inference=True)
		self.module.load_weights('backup/yolov4_module_4000.weights')
		self.module.cuda()
		self.coord_pub_module = rospy.Publisher('/comm_module_coord', Point, queue_size=1)
		self.coord_pub_board = rospy.Publisher('/board_coord', Point, queue_size=1)

		# rospy.init_node('main')
		rospy.Subscriber("/camera/color/image_raw",Image, self.callback)	 
		rospy.Subscriber("/camera/depth/points", PointCloud2, self.callback_depth)

	def callback_depth(self,value):
		self.pc_arr = ros_numpy.numpify(value)

	def my_detect(self,m,cv_img):
		use_cuda=True
		img=cv2.resize(cv_img, (m.width, m.height))
		img=cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
		boxes = do_detect(m, img, 0.4, 0.6, use_cuda)
		if len(boxes[0])==0:
			return [False,0,0,0,0]
		box=boxes[0][0]
		h,w,c=cv_img.shape
		x1 = int(box[0] * w)
		y1 = int(box[1] * h)
		x2 = int(box[2] * w)
		y2 = int(box[3] * h)
		return [True,x1,y1,x2,y2]
		
	def end_to_end(self,board,module,cv_img):
		use_cuda=True
		img=cv2.resize(cv_img, (board.width, board.height))
		img=cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
		boxes = do_detect(board, img, 0.4, 0.6, use_cuda)
		if len(boxes[0])==0:
			return [False,0,0,0,0]
		box=boxes[0][0]
		h,w,c=cv_img.shape
		x1 = int(box[0] * w)
		y1 = int(box[1] * h)
		x2 = int(box[2] * w)
		y2 = int(box[3] * h)
		
		hc=y2-y1
		wc=x2-x1

		x1=max(0,x1)
		x2=min(board.width,x2)
		y1=max(0,y1)
		y2=min(board.height,y2)
		if (x2-x1)*(y2-y1)==0 :
			return [False,0,0,0,0]
		cropped=cv2.resize(cv_img[y1:y2,x1:x2], (self.module.width,self.module.height))
		# imShow(cropped)
		cropped=cv2.cvtColor(cropped, cv2.COLOR_BGR2RGB)
		boxes=do_detect(self.module, cropped, 0.4, 0.6, use_cuda)
		if len(boxes[0])==0:
			return [False,0,0,0,0]
		box=boxes[0][0]
		# print(box,hc,wc,x1,y1)
		a1=x1+int(box[0]*wc)
		b1=y1+int(box[1]*hc)
		a2=x1+int(box[2]*wc)
		b2=y1+int(box[3]*hc)
		# print(a1,b1,a2,b2)
		return [True,a1,b1,a2,b2]

	def callback1(self,data):

		bridge = CvBridge()
		cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
		ret,x1,y1,x2,y2 = self.my_detect(self.board,cv_image)

		xc = (x1+x2)/2.0
		yc = (y1+y2)/2.0

		arr = self.pc_arr
		xpr,ypr,zpr = arr['x'][yc][xc],arr['y'][yc][xc],arr['z'][yc][xc]
		psr = PointStamped()
		psr.header.frame_id = "r200link"	# SORT OUT THE CAMERA LINK WAALA PART
		psr.header.stamp = rospy.Time(0)
		psr.point.x = zpr
		psr.point.y = -xpr
		psr.point.z = -ypr
		cord = self.listener.transformPoint("/map", psr)
		point = Point()
		point.x, point.y, point.z = cord.x, cord.y, cord.z
		self.coord_pub_board.publish(point)

	def callback2(self,data):

		bridge = CvBridge()
		cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
		ret,x1,y1,x2,y2 = self.my_detect(self.module,cv_image)

		xc = (x1+x2)/2.0
		yc = (y1+y2)/2.0

		arr = self.pc_arr
		xpr,ypr,zpr = arr['x'][yc][xc],arr['y'][yc][xc],arr['z'][yc][xc]
		psr = PointStamped()
		psr.header.frame_id = "r200link"	# SORT OUT THE CAMERA LINK WAALA PART
		psr.header.stamp = rospy.Time(0)
		psr.point.x = zpr
		psr.point.y = -xpr
		psr.point.z = -ypr
		cord = self.listener.transformPoint("/map", psr)

		point = Point()
		point.x, point.y, point.z = cord.x, cord.y, cord.z
		self.coord_pub_module.publish(point)

		cv_image=cv2.rectangle(cv_image,(x1,y1),(x2,y2),(0,0,255),2)
		cv2.imshow('frame',cv_image)
		cv2.waitKey(50)

if __name__ == '__main__':
	rospy.init_node('child_node', anonymous=True)
	child = child()
	parent = parent()

	daughter_class = daughter_class()
	r = rospy.Rate(10)
	while not rospy.is_shutdown():
		if (parent.parent_loc.x > -8):
			print('REPLACEMENT CLASS : Waiting for Mothership to Reach its destination!')
			pass
		else:
			print('REPLACEMENT CLASS : DAUGHTERSHIP TAKE OFF INITIATED!')
			time.sleep(2)
			localization = LOCALIZATION()
			while (daughter_class.armed != True):
				child.setarmc(1)
				time.sleep(1)
			child.offboard(1)
			module_detection = MODULEDETECTION()
			child.gotopose(child.child_loc.x, child.child_loc.y, 15)
			daughter_class.display_text("Daughter Takeoff Initiated")
			navlights = NAVLIGHTS()
			# rospy.Subscriber('/gazebo/link_states', LinkStates, daughter_class.handle_pose)		#COARSE TUNING
			
			threshold = THRESHOLD()
			rospy.Subscriber('/board_coord', Point, daughter_class.handle_pose)
			daughter_class.display_text("Module Detected by Drone")
			replacement_instance = replacement(threshold)
			checksum = 0
			r = rospy.Rate(10)
			time.sleep(5)
			while not rospy.is_shutdown():	# INCLUDE FLAG FOR WHILE MODULE ISNT REPLACED
				print("Y1 : ", threshold.x1)
				if( daughter_class.moduleReplaced != 1):
					if (0 < threshold.z1 < 0.5):
						if( -0.03 < threshold.x1 < 0.03):
							daughter_class.display_text("Drone aligned with Module")
							checksum += 1
							print("CHECKSUM : ", checksum)
							if (checksum > 40):
								replacement_instance.start(threshold)
								daughter_class.display_text("Enemy Module Replaced")
								time.sleep(2)
								daughter_class.display_text("DaughterShip without the module MTOW : 22.6 Kgs")
								child.gotopose(-4,-10,2)
								break
						else:
							checksum = 0
					else:
						checksum = 0
				r.sleep()
			daughter_class.display_text('IARC Simulation Challenge Accomplished!')
				

				
