#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from gazebo_msgs.msg import LinkState
import math

def sine_multiple():

    pitch_pub = rospy.Publisher('/hunter_killer_mast/pitch_controller/command',
                            Float64,
                            queue_size=10)
    roll_pub = rospy.Publisher('/hunter_killer_mast/roll_controller/command',
                            Float64,
                            queue_size=10)


    rospy.init_node('sea_state', anonymous=True)
    rate = rospy.Rate(60) # 60hz
    
    while not rospy.is_shutdown():
    
        roll_msg = 0.037*math.sin(0.5*float(rospy.get_time())+1.4) - 0.073*math.sin(0.7*float(rospy.get_time())+2.3) #Oscillation given to Red Ball in y
        
        pitch_msg = 0.029*math.sin(0.7*float(rospy.get_time())+6.3) - 0.093*math.sin(0.6*float(rospy.get_time())+4.7)

        pitch_pub.publish(pitch_msg)
        roll_pub.publish(roll_msg)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        sine_multiple()
    except rospy.ROSInterruptException:
        pass
