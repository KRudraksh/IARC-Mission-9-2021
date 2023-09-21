import cv2
import numpy as np 

from tool.darknet2pytorch import Darknet
from demo import *

board=Darknet('cfg/yolov4_box.cfg',inference=True)
board.load_weights('backup/yolov4_box_4000.weights')
board.cuda()

module=Darknet('cfg/yolov4_module.cfg',inference=True)
module.load_weights('backup/yolov4_module_4000.weights')
module.cuda()

def my_detect(m,cv_img):
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
  boxes=do_detect(module, cropped, 0.4, 0.6, use_cuda)
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

def imgcallback():
	global cv_image
	bridge = CvBridge()
	cv_image = bridge.imgmsg_to_cv2(data, "bgr8")

if __name__=='__main__':
  #img=cv2.imread('data/board.jpg')
  global cv_image
  image_sub = rospy.Subscriber("/r200/color/image_raw", Image, imgcallback)
  ret,x1,y1,x2,y2=my_detect(board,img)
  cv_image = cv2.rectangle(img,(x1,y1),(x2,y2),(0,0,255),2)
  cv2.imshow('frame',img)
  cv2.waitKey(0)