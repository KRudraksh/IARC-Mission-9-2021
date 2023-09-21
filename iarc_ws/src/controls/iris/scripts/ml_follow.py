#!/usr/bin/env python
import rospy
#from my_subscriber.msg import Person
import rospkg 
from gazebo_msgs.msg import ModelState 
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2
from time import sleep
from time import time as tm
from tool.darknet2pytorch import Darknet
from demo import *


# Modules added by controls guys
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


board=Darknet('cfg/yolov4_box.cfg',inference=True)
board.load_weights('backup/yolov4_box_4000.weights')
board.cuda()

module=Darknet('cfg/yolov4_module_new.cfg',inference=True)  
module.load_weights('backup/yolov4_module_new_4000.weights')
module.cuda()

#################################### ML detection ######################################################
def my_detect(m,cv_img):
  use_cuda=True
  img=cv2.resize(cv_img, (m.width, m.height))
  img=cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
  boxes = do_detect(m, img, 0.2, 0.6, use_cuda)
  if len(boxes[0])==0:
    return [False,0,0,0,0]
  box=boxes[0][0]
  h,w,c=cv_img.shape
  x1 = int(box[0] * w)
  y1 = int(box[1] * h)
  x2 = int(box[2] * w)
  y2 = int(box[3] * h)
  return [True,x1,y1,x2,y2]
  
def end_to_end(board,module,cv_img):
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
  cropped=cv2.resize(cv_img[y1:y2,x1:x2], (module.width,module.height))
  # imShow(cropped)
  cropped=cv2.cvtColor(cropped, cv2.COLOR_BGR2RGB)
  boxes=do_detect(module, cropped, 0.2, 0.6, use_cuda)
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

def callback(data):
    global board
    global module
    
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")

    # cv_image=cv2.resize(cv_image,(320,320))

    # ret,x1,y1,x2,y2=my_detect(board,cv_image)
    ret,x1,y1,x2,y2=my_detect(module,cv_image)
    # ret,x1,y1,x2,y2=end_to_end(board,module,cv_image)

    cv_image=cv2.rectangle(cv_image,(x1,y1),(x2,y2),(0,0,255),2)
    cv2.imshow('frame',cv_image)
    cv2.waitKey(50)
    # if time()>t0+60:
    # 	writer.release()
    # 	print('done')
    

##################################### PID #############################################################

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
        print("pidExecute successful")
        return output

def state_callback(state):
    global z_state, y_state
    pos = state.position
    y_state = pos[2]
    z_state = pos[0]

def callback1(data):
    global board
    global module

    state = rospy.Subscriber("/daughter6/joint_states", JointState, state_callback)
    
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    temp_img = bridge.imgmsg_to_cv2(data, "bgr8")

    # cv_image=cv2.resize(cv_image,(320,320))

    #ret,x1,y1,x2,y2=my_detect(board,cv_image)
    ret,x1,y1,x2,y2=my_detect(module,cv_image)
    # ret,x1,y1,x2,y2=end_to_end(board,module,cv_image)

    global cord
    ym = (x1+x2)/2
    zm = (y1+y2)/2

    print("z_centre = %f" %(zm))
    print("y_centre = %f" %(ym))
    
    cord = [ym,zm]
    
    if cord[0] == 0 :
      if cord[1] == 0 :
        cord[0] = old_cord[0]
        cord[1] = old_cord[1]
    
    old_cord[0] = cord[0]
    old_cord[1] = cord[1]

    cv2.circle(temp_img, (200,200), 1, (0,0,255), 3)
    cv2.circle(temp_img, (ym,zm), 1, (255,0,0), 3)
    cv2.rectangle(temp_img,(x1,y1),(x2,y2),(0,255,0),3)
    #cv2.imwrite('temp.jpeg', temp_img)
    #dec_img = cv2.imread('temp.jpeg')
    cv2.imshow('detected_image', temp_img)
    cv2.waitKey(1)

    # cv_image=cv2.rectangle(cv_image,(x1,y1),(x2,y2),(0,0,255),2)
    # cv2.imshow('frame',cv_image)
    # cv2.waitKey(50)
    # if time()>t0+60:
    # 	writer.release()
    # 	print('done')
    
    #kp = 0.008
    #kd = 0.003
    #ki = 0.002
    #outMin = -5
    #outMax = 5
    #iMin = -0.01
    #iMax = 0.01
    
    pid_y = PID(0.000875, 0.0, 0.000375, -0.30, 0.30, -0.001, 0.001)
    output_y = pid_y.pidExecute(200 ,cord[0])
    print("y_publish = %f" %(output_y))

    datay_pub.publish(y_state + output_y)

    #pid_z = PID(-0.000875, 0.000175, 0.000375, -0.20, 0.10, -0.001, 0.001)
    #output_z = pid_z.pidExecute(200 ,cord[1])
    #print("z_publish = %f" %(output_z))

    #dataz_pub.publish(cord[1] + output_z)

######################################################################################################

def listener():
  #global velocity_pub
  global ind 
	#rospy.init_node('main')

  image_sub = rospy.Subscriber("/daughter6/mybot/camera1/image_raw", Image, callback1)
	
  rospy.spin()
  

if __name__ == '__main__':
  
  cord = [0,0]
  old_cord = [200,200]

  rospy.init_node('ml',anonymous=True)

  datay_pub = rospy.Publisher("/daughter6/y_position_controller/command", Float64, queue_size = 10)
  dataz_pub = rospy.Publisher("/daughter6/z_position_controller/command", Float64, queue_size = 10)

  listener()