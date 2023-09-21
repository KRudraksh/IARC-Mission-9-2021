#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Range

def switch_led():
    rospy.init_node('led_switch',anonymous=True)

    
    offset=0.08

    def callback(data):
        global state
        if data.range<offset:
           
            pub1.publish(-10)
            pub2.publish(-10)


        else:
         
            pub1.publish(10)
            #print("kar naa")
            pub2.publish(10)

    
    #print("ko")
    
    pub1=rospy.Publisher('/hunter_killer_mast/led_port_controller/command',Float64,queue_size=10)
    pub2=rospy.Publisher('/hunter_killer_mast/led_starboard_controller/command',Float64,queue_size=10)
    rate=rospy.Rate(10)
    
    rospy.Subscriber('/hunter_killer_mast/proxy_sensor',Range,callback)
    rospy.spin()

    

if __name__ == '__main__':
    try:
        switch_led()
    except rospy.ROSInterruptException:
        pass
