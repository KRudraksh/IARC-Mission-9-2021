#!/usr/bin/env python
import rospy
#from my_subscriber.msg import Person
import rospkg 
from gazebo_msgs.msg import ModelState 
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, PointStamped
from std_msgs.msg import String
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2
from time import sleep
from time import time as tm
import sys
sys.path.insert(1, '/iarc_ml/pytorch-yolov4/')	# Insert absolute Path in your system to be used.
from tool.darknet2pytorch import Darknet
from demo import *

class MODULEDETECTION:
	def __init__(self):
		#global velocity_pub
		global ind 
		print("Module Detection Initiated")
		self.board=Darknet('cfg/yolov4_box.cfg',inference=True)
		self.board.load_weights('backup/yolov4_box_4000.weights')
		self.board.cuda()

		self.module=Darknet('cfg/yolov4_module.cfg',inference=True)
		self.module.load_weights('backup/yolov4_module_4000.weights')
		self.module.cuda()
		self.coord_pub_module = rospy.Publisher('/comm_module_coord', Point, queue_size=1)
		self.coord_pub = rospy.Publisher('/comm_module_coord', Point, queue_size=1)

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

	def callback(self,data):

		bridge = CvBridge()
		cv_image = bridge.imgmsg_to_cv2(data, "bgr8")

		# cv_image=cv2.resize(cv_image,(320,320))

		# ret,x1,y1,x2,y2=my_detect(self.board,cv_image)
		ret,x1,y1,x2,y2 = self.my_detect(self.module,cv_image)
		# ret,x1,y1,x2,y2=end_to_end(self.board,module,cv_image)

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
		cord = listener.transformPoint("/map", psr)

		point = Point()
		point.x, point.y, point.z = cord.x, cord.y, cord.z
		self.coord_pub_module.publish(point)

		cv_image=cv2.rectangle(cv_image,(x1,y1),(x2,y2),(0,0,255),2)
		cv2.imshow('frame',cv_image)
		cv2.waitKey(50)
		# if time()>t0+60:
		# 	writer.release()
		# 	print('done')

if __name__ == '__main__':
	rospy.init_node('ml',anonymous=True)
	MODULEDETECTION()