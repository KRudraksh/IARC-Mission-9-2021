#!/usr/bin/env python
import sys
import rospy
import std_msgs
from geometry_msgs.msg import *
from gazebo_msgs.srv import ApplyBodyWrench
import math
from Tkinter import *
import time
from pyrebase import pyrebase
from std_msgs.msg import Float64
from gazebo_msgs.msg import LinkState


config = {
  "apiKey": "AIzaSyAiBNZfad7r3YewpcqYjc4ko1u7fd2s9yE",
  "authDomain": "iarc-b9c0a.firebaseapp.com",
  "databaseURL": "https://iarc-b9c0a-default-rtdb.firebaseio.com/",
  "storageBucket": "iarc-b9c0a.appspot.com"
}

firebase = pyrebase.initialize_app(config)
storage = firebase.storage()
database = firebase.database()

def update_slider():
	global sea_state, wind_speed, wind_direction, flight_speed, mothership_endurance, daughtership_endurance
	accidents = database.child("/").get()
	data = accidents.val()

	sea_state = data.get("states")
	wind_speed = data.get("wind_speed")
	wind_direction = data.get("degs")
	flight_speed = data.get("flight_speed")
	mothership_endurance = data.get("mothership_endurance") 
	daughtership_endurance = data.get("daughtership_endurance")
	#print ("Entered update slider")

# density=1.204 #given condition
# quad_effective_area=0.2 #area hit by wind in iris
# name = 'iris::base_link' #base link name jispe lagana hai wrench
# #angle is in the world_frame
# def apply_wind(mag,angle):
# 	rospy.wait_for_service('/gazebo/apply_body_wrench')
# 	#mag is the winfd vel in 0-150 kmh
# 	theta=angle*360/100
# 	f_mag=quad_effective_area*density*mag*0.277778*5
# 	f_x=f_mag*math.cos(theta)
# 	f_y=f_mag*math.sin(theta)
# 	wind_force=rospy.ServiceProxy('/gazebo/apply_body_wrench',ApplyBodyWrench)
# 	wind_force(name,name,geometry_msgs.msg.Point(0,0,0),geometry_msgs.msg.Wrench(force=geometry_msgs.msg.Vector3(f_x,f_y,0),torque=geometry_msgs.msg.Vector3(0,0,0)),rospy.Time(secs=0,nsecs=0),rospy.Duration(secs=-1,nsecs=0))

density=1.204 #given condition
c1=1.02 #for mother
c2=1.53 #for daughter
name1 = 'iris_0::base_link' #base link name jispe lagana hai wrench
name2='typhoon_h480_1::base_link'
#angle is in the world_frame
def apply_wind(mag,angle):
	rospy.wait_for_service('/gazebo/apply_body_wrench')
	#mag is the winfd vel in 0-150 kmh
	#print(1)
	theta=angle*360/100

	f_mag1= mag*0.277778*c1
	f_x1=f_mag1*math.cos(theta)
	f_y1=f_mag1*math.sin(theta)

	f_mag2= mag*0.277778*c2
	f_x2=f_mag2*math.cos(theta)
	f_y2=f_mag2*math.sin(theta)

	wind_force=rospy.ServiceProxy('/gazebo/apply_body_wrench',ApplyBodyWrench)
	wind_force(name1,name1,geometry_msgs.msg.Point(0,0,0),geometry_msgs.msg.Wrench(force=geometry_msgs.msg.Vector3(f_x1,f_y1,0),torque=geometry_msgs.msg.Vector3(0,0,0)),rospy.Time(secs=0,nsecs=0),rospy.Duration(secs=-1,nsecs=0))
	wind_force(name2,name2,geometry_msgs.msg.Point(0,0,0),geometry_msgs.msg.Wrench(force=geometry_msgs.msg.Vector3(f_x2,f_y2,0),torque=geometry_msgs.msg.Vector3(0,0,0)),rospy.Time(secs=0,nsecs=0),rospy.Duration(secs=-1,nsecs=0))

def main():
	global sea_state, wind_speed, wind_direction, flight_speed, mothership_endurance, daughtership_endurance
	rospy.init_node('sliders_node', anonymous=True)
	sea_state_pub = rospy.Publisher('/slider/sea_state',
                            Float64,
                            queue_size=10)
	flight_speed_pub = rospy.Publisher('/slider/flight_speed',
                            Float64,
                            queue_size=10)
	mothership_endurance_pub = rospy.Publisher('/slider/mothership_endurance',
                            Float64,
                            queue_size=10)
	daughtership_endurance_pub = rospy.Publisher('/slider/daughtership_endurance',
                            Float64,
                            queue_size=10)						
	rate = rospy.Rate(100)

	
	while(not rospy.is_shutdown()):
		update_slider()
		apply_wind(wind_speed,wind_direction)  		#uncomment later
		sea_state_pub.publish(sea_state)
		flight_speed_pub.publish(flight_speed)
		mothership_endurance_pub.publish(mothership_endurance)
		daughtership_endurance_pub.publish(daughtership_endurance)
		#print(sea_state,wind_speed,wind_direction,flight_speed,mothership_endurance,daughtership_endurance)
		#os.system("rosservice call /gazebo/apply_body_wrench \'{body_name: \"iris::base_link\" , wrench: { force: { x: 50, y: 0 , z: 0.0 } }, start_time: 10000000000, duration: 1000000000 }\'")
		time.sleep(0.1)
		#rate.sleep()
		
if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
