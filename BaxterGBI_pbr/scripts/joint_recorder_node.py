#!/usr/bin/env python

import argparse

import rospy
import sys

import baxter_interface
from BaxterGBI_pbr import JointRecorder

from baxter_interface import CHECK_VERSION
from BaxterGBI_pbr.msg import record_status, modify_playback

import os
"""
joint_recorder_node

ROS node used to allow the user to record the baxter movements.
"""

#Handle stop -> stop 
def callback_recording(data):
    """
    Callback function called when a data is written on the topic, it enables/disables the recording mode.
    
    @type data.filename: string
    @param data.filename: name of the file where we want to record.
    @type req.mode: string
    @param req.mode: modality: start/stop.
    @type req.record_rate: uint16
    @param req.record_rate: rate used for recording joints' data.
    
    @returns: 0 on success, 1 on errors
    """
    rospy.loginfo("Called!!")
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.mode)
    global recorder, state, rate
    if data.mode == "stop":
        if state == 1:
           #Stop the recording
           rospy.loginfo("Stop!")
           recorder.stop()
           del recorder
           state = 0
           return 0
    elif data.mode == "start":
        if state == 0:
           rospy.loginfo("Start")
           #Start recording
           rate = rospy.Rate(data.record_rate)
           recorder = JointRecorder(data.filename,"w+", 0)
           state = 1
           return 0
    rospy.logwarn("Invalid mode!")
    return 1
           
           
def callback_modify_playback(data):
    """
    Callback function called when the playback is in pause.
    Here you can use the first part of the playbacked file and change (overwrite) the final part.
    
    
    @type data.old_filename: string
    @param data.old_filename: name of the file you want to modify.
    @type data.new_filename: string
    @param data.new_filename: name of the updated file.
    @type data.mode: string
    @param data.mode: 'start' or 'stop' the recording mode.
    @type data.line_number: uint64
    @param data.line_number: the last line you want to keep.
    @type data.record_rate: uint16
    @param data.record_rate: rate used for recording  joint's data.
    
    
    @returns: 0 on success, 1 on errors
    """
    
    
    rospy.logwarn(os.getcwd())
    
    
    rospy.loginfo("Called modify_playback!!")
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.mode)
    
    global recorder, state, rate
    if data.mode == "stop":
        if state == 1:
           #Stop the recording
           rospy.loginfo("Stop!")
           recorder.stop()
           del recorder
           state = 0
    elif data.mode == "start":
        if state == 0:
            rospy.loginfo("Start")
                   
            f1 = data.old_filename
            f2 = data.new_filename
            
            to_rename = False
            
            #TODO -> change name
            if(f1 == f2):
                to_rename = True
                f2 = "src/BaxterGBI_pbr/RecordedFile/modify_old12345.baxter" #change it!!

            file_to_copy = open(f1,"r")
            file_new = open(f2, "w+")
           
            
            line = ""
            for i in range(0,data.line_number+1):
                line = file_to_copy.readline()
                file_new.write(line)
            file_new.close()
            file_to_copy.close()
            
            #So we overwrite the old_filename
            if to_rename == True:
                os.remove(f1)
                os.rename(f2,f1)
                f2 = f1
            
            last_values = line.split(",")
            last_time_value = last_values[0]
            
            
             
            #Start recording
            rate = rospy.Rate(data.record_rate)
            recorder = JointRecorder(f2,"a", float(last_time_value))
            state = 1

def main():
    """
    Node used for the recording mode.
    """
    rospy.loginfo("Initializing node... ")
    rospy.init_node('joint_recorder_node', anonymous=True)
    rospy.Subscriber("recording_status", record_status, callback_recording)
    rospy.Subscriber("modify_playback_status", modify_playback, callback_modify_playback)
    rospy.loginfo("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    rospy.loginfo("Enabling robot... ")
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
