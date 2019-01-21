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

global rs


#Playback Service -> takes the file_name and use it for playback
def playback_handler(req):
    """Function used to provide the playback service.
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


#Record Service ->
def record_handler(req):
    """Function used to provide the record service.
    """
    print "Called !!!"
    path = "src/BaxterGBI_pbr/RecordedFile/"+req.filename
    #TODO -> Add default value for rate (100)
    record_function(path, req.record_rate)
    return 0


def list_files_handler(req):
    """Function used to provide the entire list of the recorded movements.
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
    file_path_string = "src/BaxterGBI_pbr/RecordedFile/"+req.filename
        
    if os.path.isfile(file_path_string) :
        os.remove(file_path_string)
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

    print("Enabling robot... ")
    rs.enable()

    service1 = rospy.Service('playback', Playback, playback_handler)
    service2 = rospy.Service('record', Record, record_handler)
    service3 = rospy.Service('files', ListFiles, list_files_handler)
    service4 = rospy.Service('delete_file', DeleteFile, delete_file_handler)
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
