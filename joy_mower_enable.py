#!/usr/bin/env python

#===========================================
# Christoph Sobel 27.08.2023
#
# enable mow motor for mowgli open mower
#
#===========================================

import time
import rospy
from mower_msgs.srv import MowerControlSrv
from sensor_msgs.msg import Joy
last_joy = Joy
motor_running = False

def mow_enable(motor_cmd):
    rospy.wait_for_service('mower_service/mow_enabled')
    try:
        mower_service = rospy.ServiceProxy('mower_service/mow_enabled', MowerControlSrv)
        res = mower_service(motor_cmd,0)
        # print("enable", res)
    except rospy.ServiceException as e:
        print("Service call failed: {}".format(e))

def joy_callback(joy_msg):
    global last_joy, motor_running
    if not last_joy.buttons == joy_msg.buttons:
        print("buttons: ", joy_msg.buttons)
    if joy_msg.buttons[6] and not last_joy.buttons[6]:
        print("Button 5 pressed")
        if motor_running:
            print("stopping motor")
            mow_enable(0)
            motor_running = False
        else:
            print("starting motor")
            mow_enable(1)
            motor_running = True
    last_joy = joy_msg

def listener():
    rospy.init_node('joy_mower_enable', anonymous=True)
    rospy.Subscriber("/joy", Joy, joy_callback)

    rospy.spin()

if __name__ == "__main__":
    print("enable mow motor")
    listener()
    print("done")
