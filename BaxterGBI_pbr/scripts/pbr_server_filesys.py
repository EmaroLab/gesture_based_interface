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
def pbr_server_filesys():
    """Main of the node. It makes available the services related to the file management.
    """
    print("Initializing node... ")
    rospy.init_node('pbr_server_filesys')


    service1 = rospy.Service('files', ListFiles, list_files_handler)
    service2 = rospy.Service('delete_file', DeleteFile, delete_file_handler)
    service3 = rospy.Service('rename_file', RenameFile, rename_file_handler)
    print "PBR node executed -> providing file management services."

    rospy.spin()

if __name__ == "__main__":
    pbr_server_filesys()
