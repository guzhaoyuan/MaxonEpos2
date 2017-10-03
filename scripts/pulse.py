#!/usr/bin/env python
'''
this script generate a pulse applied onto the servo to disturb the pendulum

'''
from epos2.srv import *
import rospy
import sys

service = '/applyTorque'

def request_torque(position, current, init=0):
    # print("wait for Service")
    #asset current in range(-2,2)
    rospy.wait_for_service(service)
    try:
        applyTorque = rospy.ServiceProxy(service, Torque)
        res = applyTorque(position, current, init)
        return res
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

step = 0
# while(True):
if len(sys.argv[1:]) == 1:
    arg = min(int(sys.argv[1]), 25)
    while(step < arg):
        step += 1
        res = request_torque(step, 2)
    res = request_torque(step+1, 0)
    print("pulse done")