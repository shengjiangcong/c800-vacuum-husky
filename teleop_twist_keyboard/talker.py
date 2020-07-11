#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def talker():
    pub = rospy.Publisher('/cmd_vel', Twist,queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) 
    msg = Twist()

    for i in range(1,50):
     msg.linear.x = 0.3
     msg.linear.y = 0
     msg.linear.z = 0
     msg.angular.x = 0
     msg.angular.y = 0
     msg.angular.z = 0
     pub.publish(msg)
     print i
     rate.sleep()

if __name__ == '__main__':
    #try:
        talker()
    #except rospy.ROSInterruptException:
     #   pass
