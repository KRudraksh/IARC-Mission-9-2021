import rospy
from std_msgs.msg import Float64

def switch_led():
    rospy.init_node('led_switch',anonymous=True)
    pub1=rospy.Publisher('/hunter_killer_mast/led_port_controller/command',Float64,queue_size=10)
    pub2=rospy.Publisher('/hunter_killer_mast/led_starboard_controller/command',Float64,queue_size=10)
    rate=rospy.Rate(10)
    while not rospy.is_shutdown():
        a=input("enter LED state")
        if a==0:
            pub1.publish(10)
            pub2.publish(10)
        elif a==1:
            pub1.publish(-10)
            pub2.publish(-10)
        else:
            print("invalid comand")
        rate.sleep()

switch_led()



