#!/bin/bash
# gnome-terminal --tab
#while :
#do
gnome-terminal --tab -e "bash -c 'echo ===============================; echo Starting GZWEB; echo ===============================; sleep 15s; cd ~/gzweb; npm start'"
gnome-terminal --tab -e "bash -c 'echo ===============================; echo Starting TUNNEL; echo ===============================; sleep 15s; ngrok start iarc; exit'"
gnome-terminal --tab -e "bash -c 'echo ===============================; echo Starting Mission Script; echo ===============================; python run_mission.py; exit'"
echo ===============================; echo Starting Gazebo SITL; echo ===============================; cd PX4-Autopilot; source sitl_launch.sh;
echo ===============================; echo Mission Complete; sleep 2s; echo ===============================;
#done