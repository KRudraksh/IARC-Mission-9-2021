#!/usr/bin/env python
import rospy, rostopic
import numpy as np
from mavros_msgs.msg import *
import math
from pyrebase import pyrebase
import time

config = {
  "apiKey": "AIzaSyAiBNZfad7r3YewpcqYjc4ko1u7fd2s9yE",
  "authDomain": "iarc-b9c0a.firebaseapp.com",
  "databaseURL": "https://iarc-b9c0a-default-rtdb.firebaseio.com/",
  "storageBucket": "iarc-b9c0a.appspot.com"
}

firebase = pyrebase.initialize_app(config)
storage = firebase.storage()
database = firebase.database()
accidents = database.child("/").get()
data = accidents.val()
mothership_endurance = (data.get("mothership_endurance"))*15/100 
daughtership_endurance = (data.get("daughtership_endurance"))*15/100

mother_on_daughter=1 #take from real life
xmnd=50 #Calculated manually
xmwd=30 #Calculated manually
xd=70 #Calculated manually
f1=0.1 #average time period of receiving throttle values
mother_time=0
daughter_time=0
daughter_power=0
mother_power = 0



#Function to calculate the required power capacity for given slider for mother with daughter
def Mnd(st1,x):
	curr1 = float( 1.18*x - 0.0024*x**2)
	currt = curr1*6
	capacity = currt*st1
	return capacity

#Function to calc the required power capacity for given slider for daughter
def d(st2,x):
	curr1 = float(1.18*x - 0.0024*x**2)
	currt = curr1*8
	capacity = currt*st2
	return capacity



def c_mother(data1):
	#initialiation of endurance value
	global mothership_endurance, mother_power,database, data
	MT = (data.get("mothership_endurance"))*15/100 
	mother_tc = Mnd(0.6*MT*60,xmnd)+Mnd(0.4*MT*60,xmwd)
	if (mother_tc-mother_power)>0:
		accidents = database.child("/").get()
		data = accidents.val()
		MT = (data.get("mothership_endurance"))*15/100 
		mother_tc = Mnd(0.6*MT*60,xmnd) + Mnd(0.4*MT*60,xmwd)
		curr_throttle_value = data1.throttle
		power_req = Mnd(f1,curr_throttle_value)
		mother_power += power_req
		current_mother_bat = 100*(mother_tc-mother_power*100)/mother_tc
		#battery percentage
		#print("mother :",current_mother_bat)
		if (current_mother_bat <= 100 and current_mother_bat > 0):
			database.child("/battery1").set(int(current_mother_bat))
		else:
			print("SCAM mother battery value")	
			#database.child("/battery1").set(int(current_mother_bat))
		
def c_daughter(data2):
	global daughtership_endurance, daughter_power,database, data
	DT = (data.get("daughtership_endurance"))*15/100
	daughter_tc = d(DT*60,xd)
	if (daughter_tc - daughter_power)>0:
		accidents = database.child("/").get()
		data = accidents.val()
		DT = (data.get("daughtership_endurance"))*15/100
		daughter_tc = d(DT*60,xd)
		curr_throttle_value = data2.throttle
		power_req = d(f1,curr_throttle_value)
		daughter_power += power_req
		#battery percentage
		current_daughter_bat = 100*(daughter_tc-daughter_power*100)/daughter_tc
		#print("daughter :", current_daughter_bat)
		if (current_daughter_bat <= 100 and current_daughter_bat >0):
			database.child("/battery2").set(int(current_daughter_bat))
		else:
			print("SCAM daughter battery value")	
			#database.child("/battery2").set(int(current_daughter_bat))
		
def m():
	rospy.init_node('endurance',anonymous = True)
	rospy.Subscriber('/uav0/mavros/vfr_hud',VFR_HUD,c_daughter)
	rospy.Subscriber('/uav1/mavros/vfr_hud',VFR_HUD,c_mother)
	rospy.spin()

def n():
	i = 100
	while i <= 100:
		database.child("/battery1").set(i)
		database.child("/battery2").set(i/2+40)
		i = i - 1
		#print(i)
		time.sleep(1)


if __name__ == '__main__':
	try:
		m()
		#print("Entered")
	except rospy.ROSInterruptException:
		pass
