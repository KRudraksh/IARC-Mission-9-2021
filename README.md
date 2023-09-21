# IARC 2021

Click on the below image to view the simulation video
<!-- https://www.youtube.com/watch?v=Fl70FG0QKSU -->
<div align="left">
      <a href="https://www.youtube.com/watch?v=Fl70FG0QKSU">
         <img src="https://img.youtube.com/vi/Fl70FG0QKSU/0.jpg" style="width:60%;">
      </a>
</div>

<img src="/results/iarc_mast.gif" width="30%" height="30%"/>  <img src="/results/iarc_mast_real.gif" width="30%" height="30%"/><br>
<img src="/results/replacement_ros.gif" width="60%"/>

It is broadly divided on the basis of the different subsystems used in the code. 

## Mission Broad Logic - 
The Mothership takes off along with the daughtership in tow, and starts taking laps around the Pylon, using GPS Global Waypoint navigation. After the Laps are over, the Daughtership seperates from the Mothership, takes off, and starts looking for the Mast. It does so by moving around the mast, and using Machine Learning to detect the Mast Board. It then uses OpenCV to detect the LED Navigation lights, and then orients itself according to their position and orientation. After it has a good estimate of the mast board location, it approaches the module and starts the "coarse tuning" maneuver, which enables better controls of the end effector. Then, the "fine tuning" maneuver starts, which runs at a faster rate than the "coarse tuning maneuver", and that updates just the end-effector position according to the position of the module. Both of these maneuvers take the module location from the Machine Learning Module, and use PID Controllers to controls the actuators involved. Once the Module is aligned with the end-effector for a specified minimum amount of time, the end-effector executes the "replacement" maneuver, where it grabs the mast, removes the module, and then places another one in place of it. All this while, the Mothership is returning home. It started that as soon as the daughtership seperated from it. 

## Dependencies
* PX4-Autopilot - Clone the latest PX4 firmware that contains the code of the PX4 Flight Stack, that controls the SITL Simulation in Gazebo. 
* The Fast-RTPS - Dependencies of PX4. 

## Structure

The [iarc_ws](https://github.com/atharva7am/IARC_2021/tree/main/iarc_ws) containing all of the code along with other packages.
		
### [controls](https://github.com/atharva7am/IARC_2021/tree/main/iarc_ws/src/controls)
This folder consists of the Major Autopilot code used for Guidance, Navigation and Replacement of the module, along with a few utilities that assist in the same. It also has the urdf Description packages for the Joint Controllers used in our mechanism.

* [autopilot.py](https://github.com/atharva7am/IARC_2021/blob/main/iarc_ws/src/controls/aerove_controls_tools/iarc_planning/autopilot/src/autopilot.py) - This is the master file that contains the mission logic and handles the GNC of the Mothership and the decision making logic for the whole mission.
* [child_node.py](https://github.com/atharva7am/IARC_2021/blob/main/iarc_ws/src/controls/aerove_controls_tools/iarc_planning/autopilot/src/child_node.py) - This is the file that controls the Daughtership, and the replacement maneuver. It consists of many classes, each built to suit specific purposes.

### [iarc_ml](https://github.com/atharva7am/IARC_2021/tree/main/iarc_ws/src/iarc_ml)
This folder consists of the Machine Learning code used to detect the mast and eventually the module from a respectable distance, to assist in the maneuvering and navigation of the drone towards the module. It also has all the dependencies along with a few utilities used by the code.

### [Localization](https://github.com/atharva7am/IARC_2021/tree/main/iarc_ws/src/Localization/orb_slam_2_ros)
This folder consists of the ORBSLAM2 package that is used by the daughtership for localization when it detaches from the mothership, and is close to the mast. It also has a custom python script that carries out the calculations and handles the frame conversions required to accurately predict the current position.

### [Mavlink](https://github.com/atharva7am/IARC_2021/tree/main/iarc_ws/src/mavlink) and [Mavros](https://github.com/atharva7am/IARC_2021/tree/main/iarc_ws/src/mavros)
These folders include the MAVROS package that is used to communicate with the drone. 

### [Slider](https://github.com/atharva7am/IARC_2021/tree/main/iarc_ws/src/slider)
This package consists of the code used by the Sliders in the web interface to communicate with the SITL code. 



