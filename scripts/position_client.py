#!/usr/bin/env python

from epos2.srv import *
import rospy
import sys

def request_position(position, isAbsolute=0):
    # print("wait for Service")
    rospy.wait_for_service('moveToPosition')
    try:
        # print("now request service")
        moveToPosition = rospy.ServiceProxy('moveToPosition', Position)
        res = moveToPosition(position, isAbsolute)
        return res
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [position isAbsolute(0/1)]"%sys.argv[0]

if __name__ == "__main__":

    if len(sys.argv) == 2:
        position = int(sys.argv[1])
        isAbsolute = 0
        res = request_position(position, isAbsolute)
        print res.position_new
    elif len(sys.argv) == 3:
        position = sys.argv[1]
        isAbsolute = sys.argv[2]
        print sys.argv[0]
        print sys.argv[0]
        # print position
        # print isAbsolute
        # res = request_position(position, isAbsolute)
        # print res.position_new
    else:
        print usage()
        res = request_position(0, 1)
        print "back to home from:", res.position_new
        # step = 0
        # while(True):
        #     step += 1
        #     rospy.loginfo("request:%s",step)
        #     if step % 2:
        #         res = request_position(10)
        #     else:
        #         res = request_position(20)
        #     print res.position_new