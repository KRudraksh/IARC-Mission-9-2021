import sys
import os
import time
from pyrebase import pyrebase
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

database.child("/").set({'switch':0,'power':0, 'states':0,'wind_speed':0,'degs':0,'flight_speed':50,'mothership_endurance':100,'daughtership_endurance':100,'battery1':100,'battery2':100,'time remaining':540})


while True:
	switch = 0
	accidents = database.child("/").get()
	data = accidents.val()
	switch = data.get("switch")
	if(switch == 1):
		print("autopilot started")
		time.sleep(6)
		os.system("rosservice call gazebo/unpause_physics")
		time.sleep(6)
		os.system("roslaunch autopilot mission.launch")
		break
	else:
		print('start the simulation')