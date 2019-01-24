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
    
    @type req.msg.filename: string
    @param req.msg.filename: name of the file you want to play.
    @type req.msg.loops: uint8
    @param req.msg.loops: number of times you want to play it.
    @type req.msg.scale_vel: float32
    @param req.msg.scale_vel: number to scale velocity of movement.
    
    @returns: 0 on success, 1 on errors
    """
    print "Called!!!"

    #Check if the file exists
    file_path_string = "src/BaxterGBI_pbr/RecordedFile/"+req.msg.filename
    print(file_path_string)
    if os.path.isfile(file_path_string) :
        map_file(file_path_string, req.msg.loops, req.msg.scale_vel)
        return 0
    else:
        print("The file doesn't exist!!")
        return 1


def record_start_handler(req):
    """
    Service used to start the recording mode on the baxter.
    
    @type req.filename: string
    @param req.filename: name of the recorderd file.
    @type req.record_rate: uint16
    @param req.record_rate: rate used for recording joints' data.
    
    @returns: 0 on success, 1 on errors
    """
    print "Called !!!"
     
    msg = record_status()
    
    msg.filename = "src/BaxterGBI_pbr/RecordedFile/"+req.filename
    msg.mode = "start"
    msg.record_rate = req.record_rate
    pub.publish(msg)
    return 0


def record_stop_handler(req):
    """
    Service used to stop the recording mode on the baxter.
    
    @returns: 0 on success, 1 on errors
    """
    
    print "Called !!!"
    msg = record_status()
    msg.filename = " "
    msg.mode = "stop"
    msg.record_rate = 0
    
    pub.publish(msg)
    return 0


#pbr_node initialization
def pbr_server_baxter():
    """Main of the node. It makes available the services and wait for requests.
    """
    print("Initializing node... ")
    rospy.init_node('pbr_server_baxter')
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
    print "PBR node executed -> providing playback/record services."

    def clean_shutdown():
        print("\nExiting example...")
        if not init_state:
            print("Disabling robot...")
            rs.disable()
    rospy.on_shutdown(clean_shutdown)

    rospy.spin()

if __name__ == "__main__":
    pbr_server_baxter()
