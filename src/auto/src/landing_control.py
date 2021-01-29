#!/usr/bin/env python

import rospy
import numpy as np
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import Pose,PoseStamped,Point,Quaternion, Twist,TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

from threading import Thread


waypoint = []

xreal = 0
yreal = 0
zreal = 0

ekm1x = 0
ekm1y = 0
ekm1z = 0

curr = [0,0,1.5]

takeoff = 1

def start():
    
    rate = rospy.Rate(10)

    print('Arming :')
    arm = arm_srv.(value=True)
    print(arm)

    print('\n offboard mode : ')
    mode = mode_srv(custom_mode = 'OFFBOARD')
    print(mode)

    while not rospy.is_shutdown:
        try:
            control_cmd()
            local_pos_sub = rospy.Subscriber('/mavros/local_position')
            rate.sleep()
        except:
            print("Exception at start function")

def control_cmd():
    global ekm1x,ekm1y,ekm1z

    if takeoff == 1:
        ref = takeoff(ekm1z)
        xref = ref[0]
        yref = ref[1]
        zref = ref[2]
        aref = 0

def takeoff():
    global takeoff
    if takeoff == 1:
        xref = 0
        yref = 0
        zref = 2
    
if __name__ = '__main__':
    try:
        rospy.init_node('control_node',anonymous=True)
        vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel',TwistStamped,queue_size=10)
        arm_srv = rospy.ServiceProxy('/mavros/cmd/arming',CommandBool)
        mode_srv = rospy.ServiceProxy('mavros/set_mode',SetMode)
        start()
        pos_thread = Thread(target=start,args=())
        pos_thread.daemon = True
        pos_thread.start()
    except:
        print('Exception')    
