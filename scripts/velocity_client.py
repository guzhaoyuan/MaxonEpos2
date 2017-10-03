#!/usr/bin/env python

from epos2.srv import *
import rospy
import sys

def request_velocity(velocity):
    # print("wait for Service")
    rospy.wait_for_service('moveWithVelocity')
    try:
        # print("now request service")
        moveToPosition = rospy.ServiceProxy('moveWithVelocity', Velocity)
        res = moveToPosition(velocity)
        return res
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [Velocity]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 2:
        velocity = int(sys.argv[1])
        res = request_velocity(velocity)
    else:
        print usage()

        step = 0
        while(True):
            step += 1
            rospy.loginfo("request:%s",step)
            if step % 2:
                res = request_velocity(10)
            else:
                res = request_velocity(20)