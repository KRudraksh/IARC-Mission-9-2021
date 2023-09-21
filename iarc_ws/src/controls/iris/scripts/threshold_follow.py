#!/usr/bin/env python
import rospy
#from my_subscriber.msg import Person
import rospkg
import ros_numpy
from gazebo_msgs.msg import ModelState 
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from cv_bridge import CvBridge
import numpy as np
import cv2
from time import sleep
from time import time as tm
#from demo import *


# Modules added by controls guys
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


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
    global x_state, y_state, z_state
    pos = state.position
    x_state = pos[1]
    y_state = pos[2]
    z_state = pos[0]

def callback1(data):
    #global board
    #global module

    state = rospy.Subscriber("/daughter6/joint_states", JointState, state_callback)
    
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    temp_img = bridge.imgmsg_to_cv2(data, "bgr8")

    # cv_image=cv2.resize(cv_image,(320,320))

    #ret,x1,y1,x2,y2=my_detect(board,cv_image)
    #ret,x1,y1,x2,y2=my_detect(module,cv_image)
    # ret,x1,y1,x2,y2=end_to_end(board,module,cv_image)

    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    lower = np.array([0,120,70])
    upper= np.array([10,255,255])
    mask1 = cv2.inRange(hsv, lower, upper)

    lower = np.array([175,100,70])
    upper= np.array([180,255,255])
    mask2 = cv2.inRange(hsv, lower, upper)

    mask = mask1 + mask2

    cnt, h, z = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    x,y,b,h = cv2.boundingRect(cnt)

    ym = x + b/2
    zm = y + h/2

    global cord
    cord = [ym,zm]

    print("z_centre = %f" %(zm))
    print("y_centre = %f" %(ym))
    
    #if cord[0] == 0 :
    #  if cord[1] == 0 :
    #    cord[0] = old_cord[0]
    #    cord[1] = old_cord[1]
    
    #old_cord[0] = cord[0]
    #old_cord[1] = cord[1]

    cv2.circle(temp_img, (200,200), 1, (0,0,255), 3)
    cv2.circle(temp_img, (ym,zm), 1, (255,0,0), 3)
    cv2.rectangle(temp_img,(x,y),(x+b,y+h),(0,255,0),3)
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
    
    kp = 0.008
    kd = 0.003
    ki = 0.002
    outMin = -5
    outMax = 5
    iMin = -0.01
    iMax = 0.01
    
    pid_y = PID(0.004075, 0.00, 0.000575, -0.30, 0.30, -0.001, 0.001)
    output_y = pid_y.pidExecute(200 ,cord[0])
    print("output_y = %f error = %f" %(output_y , 200 - cord[0]))
    datay_pub.publish(y_state + output_y)

    pid_z = PID(0.004075, 0.00, 0.000575, -0.20, 0.10, -0.001, 0.001)
    output_z = pid_z.pidExecute(200 ,cord[1])
    print("output_z = %f error = %f" %(output_z , 200 - cord[1]))
    dataz_pub.publish(z_state + output_z)


def callback2(data):
    arr = ros_numpy.numpify(data)
    x1, y1, z1 = arr['x'][cord[0]][cord[1]], arr['y'][cord[0]][cord[1]], arr['z'][cord[0]][cord[1]]
    
    kp = 0.4
    kd = 0.08
    ki = 0.0
    outMin = -0.20
    outMax = 0.20
    iMin = -0.01
    iMax = 0.01
    
    pid = PID(kp, ki, kd, outMin, outMax, iMin, iMax)
    output = -pid.pidExecute(0.75,z1)
    print("True distance = %f ; depth error = %f ; joint_value = %f" %(z1 ,(0.75-z1), x_state))

    try:
      datax_pub.publish(x_state + output)
    except:
      print("Error")

######################################################################################################

def listener():
  #global velocity_pub
  global ind 
	#rospy.init_node('main')

  image_sub = rospy.Subscriber("/camera/color/image_raw", Image, callback1)
  depth_sub = rospy.Subscriber("camera/depth/points", PointCloud2, callback2)
	
  rospy.spin()
  

if __name__ == '__main__':
  
  cord = [0,0]
  #old_cord = [200,200]

  rospy.init_node('threshold',anonymous=True)

  datax_pub = rospy.Publisher("/daughter6/x_position_controller/command", Float64, queue_size = 10)
  datay_pub = rospy.Publisher("/daughter6/y_position_controller/command", Float64, queue_size = 10)
  dataz_pub = rospy.Publisher("/daughter6/z_position_controller/command", Float64, queue_size = 10)

  listener()