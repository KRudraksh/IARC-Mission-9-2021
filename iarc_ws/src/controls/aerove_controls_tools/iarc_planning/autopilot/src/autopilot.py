#!/usr/bin/env python
import rospy
import time
import numpy as np
# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped
# import all mavros messages and services
from mavros_msgs.msg import PositionTarget, GlobalPositionTarget, Altitude, ParamValue, State, VFR_HUD
import mavros_msgs.msg
import std_msgs.msg
from mavros_msgs.srv import SetMode, CommandBool, ParamSet
import multiprocessing 
from threading import Thread
import thread
from utils.offboard import child, parent
from std_msgs.msg import Int32, Float64
import sys
import os
from pyrebase import pyrebase
import pymap3d as pm
# import subprocess


class autopilot:
	def __init__(self):
		self.counter = 540
		self.child = child()
		self.parent = parent()
		self.currentstate = 'STANDBY'
		self.isMastDetected = False
		self.isModuleDetected = False
		self.moduleLocation = Point()
		self.mastLocation = Point()
		self.mastLocation.x = 10
		self.mastLocation.y = 0
		self.mastLocation.z = 10
		self.v = 10
		config = {
		  "apiKey": "AIzaSyAiBNZfad7r3YewpcqYjc4ko1u7fd2s9yE",
		  "authDomain": "iarc-b9c0a.firebaseapp.com",
		  "databaseURL": "https://iarc-b9c0a-default-rtdb.firebaseio.com/",
		  "storageBucket": "iarc-b9c0a.appspot.com"
		}
		firebase = pyrebase.initialize_app(config)
		storage = firebase.storage()
		self.database = firebase.database()
		rospy.Subscriber('/slider/flight_speed', Float64, self.handle_vel)
		rospy.Subscriber('/uav0/mavros/state', State, self.handle_state)
		rospy.Subscriber('/uav1/mavros/vfr_hud', VFR_HUD, self.handle_vfr)

	def display_text(self,message):
		self.database.child("/display_text").set(message)

	def handle_vfr(self, msg):
		self.vfr = msg

	def handle_state(self, msg):
		self.daughter_armed = msg.armed

	def handle_vel(self, data):
		self.v = data.data

	def dummy_subs(self,vals):
		self.database.child("/time remaining").set(self.counter)
		if (self.counter >0):
			self.counter-=1
		if (self.counter < -360):
			self.reset_sim()
		rospy.sleep(1.1)


	def TAKEOFF(self, z):
		rospy.Subscriber('/uav1/mavros/altitude', Altitude, self.dummy_subs)
		self.parent.setarmp(1)
		self.parent.offboard(1)
		self.display_text("Mothership Takeoff Initiated!")
		self.parent.gotopose(0, 0, z)
		self.display_text("Combined Drones Specs --> MTOW : 50.3 Kgs ; Inertia(Kg m^2): Ixx = 2.1, Iyy = 19.9, Izz = 10.3")
		self.LAP()
		# self.PARENTLAND()


	def LAP(self):
		lat0 = 47.3977418
		lon0 = 8.5455940
		h0 = 0
		rospy.wait_for_service('/uav1/mavros/param/set')
		try:
			setparam = rospy.ServiceProxy('/uav1/mavros/param/set', ParamSet)
			os.system("rosservice call /uav1/mavros/param/set MPC_ACC_HOR [0,3.0]")
			os.system("rosservice call /uav1/mavros/param/set MPC_ACC_HOR_MAX [0,3.0]")
			os.system("rosservice call /uav1/mavros/param/set MPC_TILTMAX_AIR [0,40.0]")
			os.system("rosservice call /uav1/mavros/param/set LNDMC_XY_VEL_MAX [0,8.0]")
			os.system("rosservice call /uav1/mavros/param/set MPC_XY_CRUISE [0,8.0]")
			os.system("rosservice call /uav1/mavros/param/set MPC_XY_VEL_MAX [0,8.0]")
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
		# wplist = [[47.39765761,8.54567804,10],[47.39765051,8.5457109,10],[47.3976517,8.5490403,10],[47.397658728185355,8.549073930306008,10],[47.39767601073463,8.549099230295639,10],[47.39769957818326,8.54910992151855,10],[47.39771976,8.5490977,10],[47.39773617,8.54907284,10],[47.39774184,8.54903777,10],[47.39774191,8.54570904,10],[47.39773547,8.54567526,10],[47.39771924,8.54565179,10],[47.39769602,8.54564273,10],[47.39767368766275,8.545654009347942,10]]#*8
		#wplist = [[47.397660849276726, 8.545655730049234, 10.0],[47.397660796492254, 8.549073402298644, 10.0],[47.3977779, 8.5490609, 10.0],[47.39773280546047, 8.545655730133277, 10.0]]
		#wplist = [[47.397660849276726, 8.545655730049234, 10.0],[47.3976905, 8.5490682, 10.0],[47.3977351, 8.5491260, 10],[47.3977779, 8.5490609, 10.0],[47.39773280546047, 8.545655730133277, 10.0]]
		self.display_text("Mothership Journey towards the Target Commences!")
		wplist = [[47.39758889305129, 8.545708717059776, 10.000028552903913], [47.39758888407233, 8.547033394424107, 10.000946585842303], [47.39758885978659, 8.548358071786895, 10.003429628233812], [47.39758882019407, 8.549682749146731, 10.00747767670595], [47.397588765294756, 8.551007426502194, 10.013090731368916], [47.39769669237899, 8.551157656275798, 10.013805329408083], [47.39780463384505, 8.551007448612365, 10.013071894085146], [47.39780468874476, 8.549682765846493, 10.00745883791018], [47.39780472833758, 8.548358083076252, 10.003410788776858], [47.39780475262352, 8.547033400303055, 10.000927747946008], [47.39780476160253, 8.545708717528319, 10.000009714211904], [47.39769682737679, 8.545549755685666, 10.000002835794362]]*4
		self.lap = 0
		for wp in wplist:
			self.lap +=1
			if(self.lap%6 == 0):
				self.display_text("Completed Lap {} towards the Target".format(self.lap//6))
			x, y, z = pm.geodetic2enu(wp[0],wp[1],wp[2], lat0, lon0, h0)
			print(x, y, z)
			dist= np.sqrt(((self.parent.parent_loc.x-x)**2) + ((self.parent.parent_loc.y-y)**2) + ((self.parent.parent_loc.z-z)**2))
			rate = rospy.Rate(20)
			while(dist>4):
				print(dist)
				self.parent.og_gotopose(x,y,z, self.v)
				dist= np.sqrt(((self.parent.parent_loc.x-x)**2) + ((self.parent.parent_loc.y-y)**2) + ((self.parent.parent_loc.z-z)**2))
				self.display_text("Throttle: {}, Altitude: {}, Speed: {}".format(round(self.vfr.throttle*100,1), round(self.vfr.altitude-488,1), round(self.vfr.groundspeed,1)))
				rate.sleep()
			
		self.parent.check = 1
		self.DETACH()

	def DETACH(self):
		#Mother and Daughter combined should stop. and Daughter should Takeoff. 
		self.display_text('Mast Reached')
		while (self.daughter_armed != True):
				self.parent.gotopose_oreo(-10, 0, 10)
		time.sleep(15)
		# self.parent.gotopose_oreo(-10, 0, 5)
		self.REVERSELAP()
		# self.parent.setmode('POSCTL')

	def REVERSELAP(self):
		lat0 = 47.3977418
		lon0 = 8.5455940
		h0 = 0
		self.lap = 0
		rospy.wait_for_service('/uav1/mavros/param/set')
		try:
			setparam = rospy.ServiceProxy('/uav1/mavros/param/set', ParamSet)
			os.system("rosservice call /uav1/mavros/param/set MPC_ACC_HOR [0,15.0]")
			os.system("rosservice call /uav1/mavros/param/set MPC_ACC_HOR_MAX [0,15.0]")
			os.system("rosservice call /uav1/mavros/param/set MPC_TILTMAX_AIR [0,75.0]")
			os.system("rosservice call /uav1/mavros/param/set LNDMC_XY_VEL_MAX [0,30.0]")
			os.system("rosservice call /uav1/mavros/param/set MPC_XY_CRUISE [0,30.0]")
			os.system("rosservice call /uav1/mavros/param/set MPC_XY_VEL_MAX [0,30.0]")
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

		self.display_text("Mother Drones Specs --> MTOW : 24.668 Kgs ; Inertia(Kg m^2): Ixx = 10.587, Iyy = 17.685, Izz = 8.007")
		# wplist = [[47.39765761,8.54567804,10],[47.39765051,8.5457109,10],[47.3976517,8.5490403,10],[47.397658728185355,8.549073930306008,10],[47.39767601073463,8.549099230295639,10],[47.39769957818326,8.54910992151855,10],[47.39771976,8.5490977,10],[47.39773617,8.54907284,10],[47.39774184,8.54903777,10],[47.39774191,8.54570904,10],[47.39773547,8.54567526,10],[47.39771924,8.54565179,10],[47.39769602,8.54564273,10],[47.39767368766275,8.545654009347942,10]]#*8
		#wplist = [[47.397660849276726, 8.545655730049234, 10.0],[47.3976905, 8.5490682, 10.0],[47.3977351, 8.5491260, 10],[47.3977779, 8.5490609, 10.0],[47.39773280546047, 8.545655730133277, 10.0]]
		wplist = [[47.39758889305129, 8.545708717059776, 10.000028552903913], [47.39758885978659, 8.548358071786895, 10.003429628233812], [47.397588765294756, 8.551007426502194, 10.013090731368916], [47.39769669237899, 8.551157656275798, 10.013805329408083], [47.39780463384505, 8.551007448612365, 10.013071894085146], [47.39780472833758, 8.548358083076252, 10.003410788776858], [47.39780476160253, 8.545708717528319, 10.000009714211904], [47.39769682737679, 8.545549755685666, 10.000002835794362]]*4
		for wp in reversed(wplist):
			self.lap +=1
			if(self.lap%4 == 0):
				self.display_text("Completed Lap {} towards the Target".format(self.lap//4))
				rospy.sleep(1)
			x, y, z = pm.geodetic2enu(wp[0],wp[1],wp[2], lat0, lon0, h0)
			print(x, y, z)
			dist= np.sqrt(((self.parent.parent_loc.x-x)**2) + ((self.parent.parent_loc.y-y)**2) + ((self.parent.parent_loc.z-z)**2))
			rate = rospy.Rate(20)
			while(dist>4):
				print(dist)
				self.parent.og_gotopose(x,y,z, self.v)
				dist= np.sqrt(((self.parent.parent_loc.x-x)**2) + ((self.parent.parent_loc.y-y)**2) + ((self.parent.parent_loc.z-z)**2))
				self.display_text("Throttle: {}, Altitude: {}, Speed: {}".format(round(self.vfr.throttle*100,1), round(self.vfr.altitude-488,1), round(self.vfr.groundspeed,1)))
				rate.sleep()
		self.parent.check = 1
		self.PARENTLAND()

	def PARENTLAND(self):	# Lands at current position.
		self.parent.gotopose(0, 0, 12)
		self.display_text("Landing Initiated")
		self.parent.gotopose(0, 0, 8)
		self.parent.gotopose(0, 0, 5)
		self.parent.gotopose(0, 0, 1)
		self.parent.setmode('AUTO.LAND')
		print('MOTHERSHIP LANDED')
		time.sleep(2)
		daughter_class.display_text("Mothership reached home. Daughtership sacrificed itself for the greater good. Humanity was saved!")
		time.sleep(10)

		self.reset_sim()

	def reset_sim(self):
		os.system("cd")
		os.system("killall -9 ngrok")
		os.system("killall -9 npm")
		os.system("pkill gzserver")
		os.system("killall -9 gzclient")
		os.system("killall -9 gzserver")
		os.system("rosnode kill -a")
		rospy.sleep(1)
		self.database.child("/").set({'switch':0, 'power': 0, 'states':0,'wind_speed':0,'degs':0,'flight_speed':50,'mothership_endurance':100,'daughtership_endurance':100,'battery1':100,'battery2':100,'time remaining':540})
		rospy.sleep(1)
		os.system("killall -9 rosmaster")
		os.system("killall -9 roscore")
		os.system('./master_kill.sh')

if __name__ == '__main__':
	rospy.init_node('autopilot_node', anonymous=True)
	autopilot = autopilot()
	#self.database.child("/").set({'switch':0, 'states':0,'wind_speed':0,'degs':0,'flight_speed':0,'mothership_endurance':0,'daughtership_endurance':0,'battery1':100,'battery2':100,'time remaining':540})

	child = child()
	parent = parent()
	rospy.sleep(5)
	autopilot.TAKEOFF(10)
  