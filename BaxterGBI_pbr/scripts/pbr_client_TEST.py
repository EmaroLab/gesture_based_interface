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

def call_record(filename, record_rate):
    """Call the record function provided by pbr_server."""
    rospy.wait_for_service('record')
    try:
        record = rospy.ServiceProxy('record', Record)
        isOk = record(filename, record_rate)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def call_list_files():

    rospy.wait_for_service('files')
    try:
        files = rospy.ServiceProxy('files', ListFiles)
        n, list_files, isOK = files()
        print(list_files)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def call_delete_file(filename):

    rospy.wait_for_service('delete_file')
    try:
        deleteFile = rospy.ServiceProxy('delete_file', DeleteFile)
        isOk = deleteFile(filename)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    """Acquire paramater specified in the command line and based on them ask for the specified service."""
    if len(sys.argv) >= 2:
        type_service = int(sys.argv[1])
        if type_service == 1: #Playback
           filename = sys.argv[2]
           loops = int(sys.argv[3])
           scale_vel = int(sys.argv[4])
           print "Client: Calling Service Playback"
           call_playback(filename,loops,scale_vel)
           print "Client: Called!"
        elif type_service == 2: #Record
           filename = sys.argv[2]
           record_rate = int(sys.argv[3])
           print "Client: Calling Service Record"
           call_record(filename,record_rate)
           print "Client: Called!"
        elif type_service == 3: #List Files
           call_list_files()
        elif type_service == 4: #Remove file
            filename = sys.argv[2]
            call_delete_file(filename)
    else:
        print "Invalid input (have to pass type_service filename loops_number or rate)"
	sys.exit(1)
