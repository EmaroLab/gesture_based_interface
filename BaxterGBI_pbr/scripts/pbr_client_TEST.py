#!/usr/bin/env python

import sys
import rospy
from BaxterGBI_pbr.srv import *
from BaxterGBI_pbr.msg import *


import os


"""
Node used to test the pbr_server.
Based on the input it will ask for record or playback service from the pbr_server.
"""

def call_playback(filename, loops, scale_vel):
    """Call the playback function provided by pbr_server."""
    rospy.wait_for_service('playback')
    try:
        playback = rospy.ServiceProxy('playback', Playback)
        msg = playback_msg()
        msg.filename = filename
        msg.loops = loops
        msg.scale_vel = scale_vel
        isOk = playback(msg)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def call_record(filename, record_rate, mode):
    """Call the record function provided by pbr_server."""
    if mode == 1:
        rospy.wait_for_service('record_start')
        try:
            record_start = rospy.ServiceProxy('record_start', RecordStart)
            isOk = record_start(filename, record_rate)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
    elif mode == 0:
        rospy.wait_for_service('record_stop')
        try:
            record_stop = rospy.ServiceProxy('record_stop', RecordStop)
            isOk = record_stop()
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        

def call_list_files():

    rospy.wait_for_service('files')
    try:
        files = rospy.ServiceProxy('files', ListFiles)
        list_files = files()
        print(list_files.list_files)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def call_delete_file(filename):

    rospy.wait_for_service('delete_file')
    try:
        deleteFile = rospy.ServiceProxy('delete_file', DeleteFile)
        response = deleteFile(filename)
        if response.isError == 0:
            print("Deletion Completed!")
        else:
            print("Error during deletion")
        
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        

def call_rename_file(old,new):

    rospy.wait_for_service('rename_file')
    try:
        renameFile = rospy.ServiceProxy('rename_file', RenameFile)
        response = renameFile(old,new)
        if response.isError == 0:
            print("File renamed correctly!")
        else:
            print("Error during renaming")
        
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    """Acquire paramater specified in the command line and based on them ask for the specified service."""
    type_service = int(sys.argv[1])
    if type_service == 1: #Playback
       filename = sys.argv[2]
       loops = int(sys.argv[3])
       scale_vel = int(sys.argv[4])
       print "Client: Calling Service Playback"
       call_playback(filename,loops,scale_vel)
       print "Client: Called!"
    elif type_service == 2: #Record Start
       filename = sys.argv[2]
       record_rate = int(sys.argv[3])
       print "Client: Calling Service Start Record"
       call_record(filename,record_rate,1)
       print "Client: Called!"
    elif type_service == 3: #Record Stop
       print "Client: Calling Service Stop Record"
       call_record("",100,0)
       print "Client: Called!"
    elif type_service == 4: #List Files
       call_list_files()
    elif type_service == 5: #Remove file
        filename = sys.argv[2]
        call_delete_file(filename)
    elif type_service == 6: #Rename file
        old_filename = sys.argv[2]
        new_filename = sys.argv[3]
        call_rename_file(old_filename,new_filename)
	sys.exit(1)
