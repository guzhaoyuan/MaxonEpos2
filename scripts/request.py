#!/usr/bin/env python

'''
    request.py
    Purpose: used for test service on ros

    @author Zhaoyuan Gu
    @version 0.1 08/23/17 
'''
from epos2.srv import *
import rospy
import sys

def print_position_n_torque(x, y):
    # print("wait for Service")
    rospy.wait_for_service('moveToPosition')
    try:
        # print("now request service")
        moveToPosition = rospy.ServiceProxy('moveToPosition', Torque)
        res = moveToPosition(x, y)
        return res
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [position torque]"%sys.argv[0]

if __name__ == "__main__":
    # if len(sys.argv) == 3:
    #     position = int(sys.argv[1])
    #     torque = int(sys.argv[2])
    #     print(position,torque)
    # else:
    #     print usage()
    #     sys.exit(1)

    step = 0
    while(True):
        # init the state by call env.reset(), getting the init state from the service
        # calculate the next move
        # call step service
        step += 1
        rospy.loginfo("request:%s",step)
        if step % 2:
            res = print_position_n_torque(10, step)
        else:
            res = print_position_n_torque(20, step)
        print res.position_new
        # after getting the responce, calc the next move and call step service again