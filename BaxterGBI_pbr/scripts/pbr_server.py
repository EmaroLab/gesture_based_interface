#!/usr/bin/env python

"""
Baxter PBR node -> provides services for: playback and record baxter movements.
"""

import argparse
import sys

import rospy

import baxter_interface

import os.path

from os import listdir
from BaxterGBI_pbr.msg import *
from baxter_interface import CHECK_VERSION
from BaxterGBI_pbr.srv import *
from pbr_header import *


def playback_handler(req):
    """
    Service used to activate the playback mode on the baxter.
    """
    print "Called!!!"

    #Check if the file exists
    file_path_string = "src/BaxterGBI_pbr/RecordedFile/"+req.msg.filename
    print(file_path_string)
    if os.path.isfile(file_path_string) :
    	#TODO -> Add default value for loops (1)
        map_file(file_path_string, req.msg.loops, req.msg.scale_vel)
        return 0
    else:
        print("The file doesn't exist!!")
        return 1


def record_start_handler(req):
    """Service used to start the recording.
    """
    print "Called !!!"
     
    msg = record_status()
    
    msg.filename = "src/BaxterGBI_pbr/RecordedFile/"+req.filename
    msg.mode = "start"
    msg.record_rate = req.record_rate
    pub.publish(msg)
    return 0


def record_stop_handler(req):
    """Service used to stop the recording.
    """
    print "Called !!!"
    msg = record_status()
    msg.filename = " "
    msg.mode = "stop"
    msg.record_rate = 0
    
    pub.publish(msg)
    return 0


def list_files_handler(req):
    """Service used to provide the entire list of the recorded files.
    """
    print("Called!!")

    resp = ListFilesResponse()
    
    path = "src/BaxterGBI_pbr/RecordedFile"
    files = os.listdir(path)

    resp.n_files = len(files)
    files_name = list()

    for file in files:
        if file.endswith(".baxter"):
            path, filename = os.path.split(file)
            files_name.append(filename)
    
    resp.list_files = files_name
    resp.isError = 0
    return resp

def delete_file_handler(req):
    """
    Service used to delete a baxter file.
    """
    
    file_path_string = "src/BaxterGBI_pbr/RecordedFile/"+req.filename
        
    if os.path.isfile(file_path_string) :
        os.remove(file_path_string)
    return 0
    
def rename_file_handler(req):
    """
    Service used to rename a baxter file.
    """
    
    path = "src/BaxterGBI_pbr/RecordedFile/"
    
    if os.path.isfile(path+req.old_filename) :
        os.rename(path+req.old_filename,path+req.new_filename)
    else:
        print("There is no file with this name!")
    return 0

#pbr_node initialization
def pbr_server():
    """Main of the node. It makes available the services and wait for requests.
    """
    print("Initializing node... ")
    rospy.init_node('pbr_server')
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled

    #TODO -> check the queue size
    global pub
    pub = rospy.Publisher('recording_status', record_status, queue_size=5)

    print("Enabling robot... ")
    rs.enable()

    service1 = rospy.Service('playback', Playback, playback_handler)
    service2 = rospy.Service('record_stop', RecordStop, record_stop_handler)
    service3 = rospy.Service('record_start', RecordStart, record_start_handler)
    service4 = rospy.Service('files', ListFiles, list_files_handler)
    service5 = rospy.Service('delete_file', DeleteFile, delete_file_handler)
    service6 = rospy.Service('rename_file', RenameFile, rename_file_handler)
    print "PBR node executed -> providing services."

    def clean_shutdown():
        print("\nExiting example...")
        if not init_state:
            print("Disabling robot...")
            rs.disable()
    rospy.on_shutdown(clean_shutdown)

    rospy.spin()

if __name__ == "__main__":
    pbr_server()
