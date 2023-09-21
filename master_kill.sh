#!/bin/bash
echo killing sab kuch
killall -9 ngrok
killall -9 npm
killall -9 gzserver
killall -9 gzclient
rosnode kill -a
killall -9 rosmaster
killall -9 roscore