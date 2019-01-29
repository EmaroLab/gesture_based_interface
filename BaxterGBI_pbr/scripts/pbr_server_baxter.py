#!/usr/bin/env python

"""
Baxter PBR node -> provides services for: playback and record baxter movements.
"""

import argparse
import sys

import rospy

from baxter_interface import Gripper

import os.path

from os import listdir
from BaxterGBI_pbr.msg import *
from baxter_interface import CHECK_VERSION
from BaxterGBI_pbr.srv import *
from pbr_header import *
from ik_header import *


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
    rospy.loginfo("Called!!!")

    #Check if the file exists
    file_path_string = "src/BaxterGBI_pbr/RecordedFile/"+req.msg.filename
    rospy.loginfo(file_path_string)
    if os.path.isfile(file_path_string) :
        map_file(file_path_string, req.msg.loops, req.msg.scale_vel)
        return 0
    else:
        rospy.logerr("The file doesn't exist!!")
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
    rospy.loginfo("Called !!!")
     
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
    
    rospy.loginfo("Called !!!")
    msg = record_status()
    msg.filename = " "
    msg.mode = "stop"
    msg.record_rate = 0
    
    pub.publish(msg)
    return 0


def gripper_handler(req):
    """
    Service used to open or close the gripper.
    
    @type req.limb: string
    @param req.limb: "left" or "right" arm.
    @type req.open: uint8
    @param req.open: value from 0 (close) to 100 (open).
    
    @returns: 0 on success, 1 on errors
    """

    global left_gripper, right_gripper
    
    if req.limb == "left":
        if req.open >= 0 and req.open <= 100:
            left_gripper.command_position(req.open)
    elif req.limb == "right":
        if req.open >= 0 and req.open <= 100:
            right_gripper.command_position(req.open)
    else:
        return 1 #Invalid choice
        
    return 0


def reach_goal_handler(req):
    """
    Service used to reach a specified goal.
    
    @type req.limb: string
    @param req.limb: "left" or "right" arm.
    @type req.position: float64[]
    @param req.position: position of the goal (x, y, z).
    @type req.quaternion: float64[]
    @param req.quaternion: orientation of the goal (x, y, z, w).
    
    @returns: 0 on success, 1 on errors
    """
    
    if req.limb == "left" or req.limb == "right":
        arm = Limb(req.limb)
        #Define the final goal
        pos = []
        pos.append(req.position[0])
        pos.append(req.position[1])
        pos.append(req.position[2])

        orient = []

        orient.append(req.quaternion[0])
        orient.append(req.quaternion[1])
        orient.append(req.quaternion[2])
        orient.append(req.quaternion[3])
        
        try:
            # reformat the solution arrays into a dictionary
            joint_solution = ik_tracking(req.limb,pos,orient)   #joint_solution is an object type ReturnValue
            
            if joint_solution.isError == 1:
                rospy.logwarn("Cannot reach the goal")
            else:
                # set arm joint positions to solution
                arm.move_to_joint_positions(joint_solution.limb_joints)
        except rospy.ServiceException, e:
            rospy.logerr("Error in Inverse Kinematic problem")

#pbr_node initialization
def pbr_server_baxter():
    """Main of the node. It makes available the services and wait for requests.
    """
    rospy.loginfo("Initializing node... ")
    rospy.init_node('pbr_server_baxter')
    rospy.loginfo("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled
    
    global left_gripper, right_gripper
    
    left_gripper = baxter_interface.Gripper('left', CHECK_VERSION)
    right_gripper = baxter_interface.Gripper('right', CHECK_VERSION)
    
    global pub
    pub = rospy.Publisher('recording_status', record_status, queue_size=2)

    rospy.loginfo("Enabling robot... ")
    rs.enable()

    service1 = rospy.Service('playback', Playback, playback_handler)
    service2 = rospy.Service('record_stop', RecordStop, record_stop_handler)
    service3 = rospy.Service('record_start', RecordStart, record_start_handler)
    service4 = rospy.Service('gripper', Gripper, gripper_handler)
    service5 = rospy.Service('reach_goal', ReachGoal, reach_goal_handler)
    rospy.loginfo("PBR node executed -> providing playback/record services.")

    def clean_shutdown():
        rospy.loginfo("\nExiting example...")
        if not init_state:
            rospy.loginfo("Disabling robot...")
            rs.disable()
    rospy.on_shutdown(clean_shutdown)

    rospy.spin()

if __name__ == "__main__":
    pbr_server_baxter()
