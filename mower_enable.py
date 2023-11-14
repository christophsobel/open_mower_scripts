#!/usr/bin/env python

#===========================================
# Christoph Sobel 20.08.2023
#
# enable mow motor for mowlgi open mower
#
#===========================================

import time
import rospy
from mower_msgs.srv import MowerControlSrv

def mow_enable():
    rospy.wait_for_service('mower_service/mow_enabled')
    try:
        mower_service = rospy.ServiceProxy('mower_service/mow_enabled', MowerControlSrv)
        res = mower_service(1,0)
        # print("enable", res)
    except rospy.ServiceException as e:
        print("Service call failed: {}".format(e))

if __name__ == "__main__":
    print("enable mow motor")
    while not rospy.is_shutdown():
        mow_enable()
        time.sleep(0.25)
    print("done")
