#!/usr/bin/env python

from epos2.srv import *
import rospy
import sys

def request_torque(position, current):
    # print("wait for Service")
    rospy.wait_for_service('applyTorque')
    try:
        # print("now request service")
        applyTorque = rospy.ServiceProxy('applyTorque', Current)
        res = applyTorque(position, current)
        return res
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [position torque]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 2:
        torque = int(sys.argv[1])
        res = request_torque(0, torque)
        print "position_new:", res.position_new, "velocity:", res.velocity, "current:", res.current
    else:
        step = 0
        while(True):
            # init the state by call env.reset(), getting the init state from the service
            # calculate the next move
            # call step service
            step += 1
            # rospy.loginfo("request:%s",step)
            # if step % 2:
            #     res = request_torque(step, 10)
            # else:
            #     res = request_torque(step, 20)
            res = request_torque(step, 0)
            rospy.loginfo("position_new:", res.position_new, "velocity:", res.velocity, "current:", res.current)
            # after getting the responce, calc the next move and call step service again