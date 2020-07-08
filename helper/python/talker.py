#!/usr/bin/env python
import rospy, sys, numpy as np
import moveit_commander
from copy import deepcopy
from geometry_msgs.msg import Twist
import moveit_msgs.msg
from sensor_msgs.msg import Image
from ur5_notebook.msg import Tracker
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from time import sleep
tracker = Tracker()
def talker():
        track = rospy.Publisher('switch', Tracker, queue_size=1)
        rospy.init_node('vacuum_control',anonymous=True)   
        r = rospy.Rate(10) 
        while not rospy.is_shutdown():   
            tracker.flag2 = 1
            track.publish(tracker)
            r.sleep()
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:pass


