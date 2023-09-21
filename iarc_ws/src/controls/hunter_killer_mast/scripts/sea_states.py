#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from gazebo_msgs.msg import LinkState
import math


def sea_motion():
    
    sea_state = input("Enter sea_states code > ")

    pitch_pub = rospy.Publisher('/hunter_killer_mast/pitch_controller/command',
                            Float64,
                            queue_size=10)
    roll_pub = rospy.Publisher('/hunter_killer_mast/roll_controller/command',
                            Float64,
                            queue_size=10)
    pankha_pub = rospy.Publisher('/hunter_killer_mast/pankha_controller/command',
                            Float64,
                            queue_size=10)


    rospy.init_node('sea_state', anonymous=True)
    rate = rospy.Rate(60) # 60hz
    
    while not rospy.is_shutdown():
        if sea_state == 0:
            roll_msg = 0.005*math.sin(0.1*float(rospy.get_time())+1.4) - 0.007*math.sin(0.2*float(rospy.get_time())+2.3) #Oscillation given to Red Ball in y
            
            pitch_msg = 0.009*math.sin(0.17*float(rospy.get_time())+6.3) - 0.007*math.sin(0.23*float(rospy.get_time())+4.7)

            pitch_pub.publish(pitch_msg)
            roll_pub.publish(roll_msg)
            pankha_pub.publish(1)

            rate.sleep()
        
        if sea_state == 1:
            roll_msg = 0.037*math.sin(0.34*float(rospy.get_time())+1.4) - 0.028*math.sin(0.42*float(rospy.get_time())+2.3) #Oscillation given to Red Ball in y
            
            pitch_msg = 0.024*math.sin(0.41*float(rospy.get_time())+6.3) - 0.053*math.sin(0.52*float(rospy.get_time())+4.7)

            pitch_pub.publish(pitch_msg)
            roll_pub.publish(roll_msg)
            pankha_pub.publish(1)

            rate.sleep()
        
        if sea_state == 2:
            roll_msg = 0.047*math.sin(0.45*float(rospy.get_time())+1.4) - 0.063*math.sin(0.56*float(rospy.get_time())+2.3) #Oscillation given to Red Ball in y
            
            pitch_msg = 0.031*math.sin(0.7*float(rospy.get_time())+6.3) - 0.073*math.sin(0.49*float(rospy.get_time())+4.7)

            pitch_pub.publish(pitch_msg)
            roll_pub.publish(roll_msg)
            pankha_pub.publish(1)

            rate.sleep()
        
        if sea_state == 3:
            roll_msg = 0.055*math.sin(0.5*float(rospy.get_time())+1.4) - 0.073*math.sin(0.7*float(rospy.get_time())+2.3) #Oscillation given to Red Ball in y
            
            pitch_msg = 0.038*math.sin(0.7*float(rospy.get_time())+6.3) - 0.093*math.sin(0.6*float(rospy.get_time())+4.7)

            pitch_pub.publish(pitch_msg)
            roll_pub.publish(roll_msg)
            pankha_pub.publish(1)

            rate.sleep()



if __name__ == '__main__':
    try:
        sea_motion()
    except rospy.ROSInterruptException:
        pass
