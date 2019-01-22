#!/usr/bin/env python

import argparse

import rospy
import sys

import baxter_interface
from BaxterGBI_pbr import JointRecorder

from baxter_interface import CHECK_VERSION
from BaxterGBI_pbr.msg import record_status

#0 -> stop   1 -> recording


#Handle stop -> stop 
def callback(data):
    print("Called!!")
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.mode)
    global recorder, state, rate
    if data.mode == "stop":
        if state == 1:
           #Stop the recording
           print("Stop!")
           recorder.stop()
           del recorder
           state = 0
    elif data.mode == "start":
        if state == 0:
           print("Start")
           #Start recording
           rate = rospy.Rate(data.record_rate)
           recorder = JointRecorder(data.filename)
           state = 1

    
def main():
    """
    """
    print("Initializing node... ")
    rospy.init_node('joint_recorder_node', anonymous=True)

    rospy.Subscriber("recording_status", record_status, callback)
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()
    global state, recorder, rate
    rate = rospy.Rate(100)
    state = 0
    while not rospy.is_shutdown():
        if state == 1:
            recorder.record_instance()
        rate.sleep()


if __name__ == '__main__':
    main()
