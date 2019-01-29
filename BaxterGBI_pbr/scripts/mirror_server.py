#!/usr/bin/env python

"""
Baxter Mirror Node -> provides the mirroring service.
"""

import argparse
import sys

import struct
import rospy

from BaxterGBI_pbr.msg import *
from baxter_interface import CHECK_VERSION
from BaxterGBI_pbr.srv import *
from BaxterGBI_pbr import *

import tf

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)


global init_pose_hand, init_orient_hand, calibrated, init_pose_baxter, init_orient_baxter
calibrated = False
init_pose_hand = []
init_orient_hand = []
init_pose_baxter = []
init_orient_baxter = []


def calibrate(req):
    """
    Service used to initialize the initial posture of the baxter end effector.
    
    
    @type req.limb: string
    @param req.limb: "left" or "right" arm.
    @type data.position: float[]
    @param data.position: position we want to achieve.
    @type data.quaternion: float[]
    @param data.quaternion: orientation we want to achieve (Quaternion).
    """
    
    global arm, calibrated, init_pose_hand, init_orient_hand, init_pose_baxter, init_orient_baxter, first_data
        
    if req.limb == "left" or req.limb == "right":
        limb = req.limb
        arm = Limb(limb)  
        
        
        #Acquire initial position/orientation of the human hand (The first value published)
        init_pose_hand.append(req.position[0])
        init_pose_hand.append(req.position[1])
        init_pose_hand.append(req.position[2])
        init_orient_hand.append(req.quaternion[0])
        init_orient_hand.append(req.quaternion[1])
        init_orient_hand.append(req.quaternion[2])
        init_orient_hand.append(req.quaternion[3])
        
        resp = arm.endpoint_pose()
        #Acquire initial position/orientation of the baxter
        init_pose_baxter = resp['position']
        init_orient_baxter = resp['orientation']
        rospy.loginfo("Pose: "+ str(init_pose_baxter))
        rospy.loginfo("Orient: "+str(init_orient_baxter))
        rospy.loginfo("Calibration completed!")
        calibrated = True
        return 0    
    else:
        rospy.logerr("Invalid limb value")
        return 1
        
   
def mirror_callback(data):
    """
    Callback function associated with the topic 'mirror_end_effector'.
    Whenever a data is written in the topic, this function is called and obtain from ik_tracking function the joints values to assign and
    move the end effector to the goal.
    
    @type data.position: float[]
    @param data.position: position we want to achieve.
    @type data.quaternion: float[]
    @param data.quaternion: orientation we want to achieve (Quaternion).
    """
    
    
    
    global arm, init_pose_hand, init_orient_hand, init_pose_baxter, init_orient_baxter, first_data
    
    if calibrated == False:
        rospy.logwarn("You need to calibrate first!")
        return
        
    #Evaluate the relative movement of the hand
    pos = []
    pos.append(init_pose_baxter[0] + (data.position[0] - init_pose_hand[0]))
    pos.append(init_pose_baxter[1] + (data.position[1] - init_pose_hand[1]))
    pos.append(init_pose_baxter[2] + (data.position[2] - init_pose_hand[2]))
    
    
    orient = []
    
    #quaternion = tf.transformations.quaternion_from_euler(-3.127816, 0.000416, -1.900463)
    #type(pose) = geometry_msgs.msg.Pose
    #orient.append(quaternion[0])
    #orient.append(quaternion[1])
    #orient.append(quaternion[2])
    #orient.append(quaternion[3])
    
    orient.append(init_orient_baxter[0] + (data.quaternion[0] - init_orient_hand[0]))
    orient.append(init_orient_baxter[1] + (data.quaternion[1] - init_orient_hand[1]))
    orient.append(init_orient_baxter[2] + (data.quaternion[2] - init_orient_hand[2]))
    orient.append(init_orient_baxter[3] + (data.quaternion[3] - init_orient_hand[3]))
    
    rospy.loginfo("Start q: "+str(arm.joint_angles()))
    
    
    try:
        # reformat the solution arrays into a dictionary
        joint_solution = ik_tracking(limb,pos,orient)   #joint_solution is an object type ReturnValue
        
        if joint_solution.isError == 1:
            rospy.logwarn("Cannot reach the goal")
        else:
            # set arm joint positions to solution
            arm.move_to_joint_positions(joint_solution.limb_joints)
    except rospy.ServiceException, e:
        rospy.logerr("Error during Inverse Kinematic problem")
   


def mirror_server():
    """
    Main of the node. Takes the information from the topic and move the baxter end effector based on those values.
    """
    
    rospy.loginfo("Initializing node... ")
    rospy.init_node('mirror_server')
    rospy.loginfo("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled
    rospy.loginfo("Enabling robot... ")
    rs.enable()

    #Mirror service -> input on/off
    #service1 = rospy.Service('mirror', Mirror, mirror_handler)
    rospy.loginfo("Mirror Server executed -> mirror service available.")


    
    rospy.Subscriber("mirror_end_effector", mirror_end_effector, mirror_callback)
    
    def clean_shutdown():
        rospy.loginfo("\nExiting example...")
        if not init_state:
            rospy.loginfo("Disabling robot...")
            rs.disable()
    rospy.on_shutdown(clean_shutdown)

    rospy.spin()

if __name__ == "__main__":
    mirror_server()
