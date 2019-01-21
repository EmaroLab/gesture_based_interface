#!/usr/bin/env python

import sys
import rospy
from BaxterGBI_pbr.srv import *



import os


"""
Node used to test the pbr_server.
Based on the input it will ask for record or playback service from the pbr_server.
"""

def call_playback(filename, loops):
    """Call the playback function provided by pbr_server."""
    rospy.wait_for_service('playback')
    try:
        playback = rospy.ServiceProxy('playback', Playback)
        playback(filename, loops)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def call_record(filename, record_rate):
    """Call the record function provided by pbr_server."""
    rospy.wait_for_service('record')
    try:
        record = rospy.ServiceProxy('record', Record)
        record(filename, record_rate)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def call_list_files():

    rospy.wait_for_service('files')
    try:
        files = rospy.ServiceProxy('files', ListFiles)
        list_files = files()
        print(list_files)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


if __name__ == "__main__":
    """Acquire paramater specified in the command line and based on them ask for the specified service."""
    if len(sys.argv) >= 2:
        type_service = int(sys.argv[1])
        if type_service == 1: #Playback
           filename = sys.argv[2]
           loops = int(sys.argv[3])
           print "Client: Calling Service Playback"
           call_playback(filename,loops)
           print "Client: Called!"
        elif type_service == 2: #Record
           filename = sys.argv[2]
           record_rate = int(sys.argv[3])
           print "Client: Calling Service Record"
           call_record(filename,record_rate)
           print "Client: Called!"
        elif type_service == 3:
           call_list_files()
    else:
        print "Invalid input (have to pass type_service filename loops_number or rate)"
	sys.exit(1)
