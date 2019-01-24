#!/usr/bin/env python

"""
Baxter File Management node -> provides: list of records, rename, delete.
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
    
    @returns: n_files -> number of records.
    @returns: list_files -> array of file name.
    @returns: isError -> 0 on success, 1 on error.
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
    
    @type req.filename: string
    @param req.filename: name of the file you want to delete.
    
    @returns: isError -> 0 on success, 1 on error.
    """
    
    file_path_string = "src/BaxterGBI_pbr/RecordedFile/"+req.filename
        
    if os.path.isfile(file_path_string) :
        os.remove(file_path_string)
    return 0
    
def rename_file_handler(req):
    """
    Service used to rename a baxter file.
    
    @type req.old_filename: string
    @param req.old_filename: name of the file you want to rename.
    @type req.new_filename: string
    @param req.new_filename: new file name.
    
    @returns: isError -> 0 on success, 1 on error.
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
