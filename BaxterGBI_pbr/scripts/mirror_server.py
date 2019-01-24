#!/usr/bin/env python

"""
Baxter Mirror Node -> provides the mirroring service.
"""

import argparse
import sys

import struct
import rospy

from baxter_interface import *

from BaxterGBI_pbr.msg import *
from baxter_interface import CHECK_VERSION
from BaxterGBI_pbr.srv import *
from pbr_header import *

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


class ReturnValue(object):
    def __init__(self, limb_joints, isError):
        self.limb_joints = limb_joints
        self.isError = isError




#limb -> left or right
#parameterized poses
def ik_tracking(limb, pos, orient):
    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    poses = {
        'left': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x = pos[0],
                    y = pos[1],
                    z = pos[2],
                ),
                orientation=Quaternion(
                    x = orient[0],
                    y = orient[1],
                    z = orient[2],
                    w = orient[3],
                ),
            ),
        ),
        'right': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=0,
                    y=-0,
                    z=0,
                ),
                orientation=Quaternion(
                    x=0.367048116303,
                    y=0.885911751787,
                    z=-0.108908281936,
                    w=0.261868353356,
                ),
            ),
        ),
    }

    ikreq.pose_stamp.append(poses[limb])
    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 1

    # Check if result valid, and type of seed ultimately used to get solution
    # convert rospy's string representation of uint8[]'s to int's
    resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
                               resp.result_type)
                               
    if (resp_seeds[0] != resp.RESULT_INVALID):
        seed_str = {
                    ikreq.SEED_USER: 'User Provided Seed',
                    ikreq.SEED_CURRENT: 'Current Joint Angles',
                    ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                   }.get(resp_seeds[0], 'None')

        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        print "\nIK Joint Solution:\n", limb_joints
        print "------------------"
        print "Response Message:\n", resp

        return ReturnValue(limb_joints,0)
    else:
        print("INVALID POSE - No Valid Joint Solution Found.")

    return ReturnValue(None,1)




def mirror_callback(data):
    """
    """
    #Acquire data from the topic
    pos = []
    pos.append(data.position[0])
    pos.append(data.position[1])
    pos.append(data.position[2])
    
    
    orient = []
    
    #quaternion = tf.transformations.quaternion_from_euler(-3.127816, 0.000416, -1.900463)
    #type(pose) = geometry_msgs.msg.Pose
    #orient.append(quaternion[0])
    #orient.append(quaternion[1])
    #orient.append(quaternion[2])
    #orient.append(quaternion[3])
    
    orient.append(data.quaternion[0])
    orient.append(data.quaternion[1])
    orient.append(data.quaternion[2])
    orient.append(data.quaternion[3])
    
    #TODO -> make as parameter
    limb = "left"
    
    arm = Limb(limb)
    print("Start q: "+str(arm.joint_angles()))
    
    
    try:
        # reformat the solution arrays into a dictionary
        joint_solution = ik_tracking(limb,pos,orient)   #joint_solution is an object type ReturnValue
        
        if joint_solution.isError == 1:
            print("Cannot reach the goal")
        else:
            # set arm joint positions to solution
            arm = Limb(limb)
            arm.move_to_joint_positions(joint_solution.limb_joints)
    except rospy.ServiceException, e:
        print("Ahahah speravi andasse bene!!")
   
   
def mirror_handler(req):
   """
   """



def mirror_server():
    """Main of the node. It makes available the services and wait for requests.
    """
    print("Initializing node... ")
    rospy.init_node('mirror_server')
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled
    print("Enabling robot... ")
    rs.enable()

    #Mirror service -> input on/off
    #service1 = rospy.Service('mirror', Mirror, mirror_handler)
    print "Mirror Server executed -> mirror service available."


    
    rospy.Subscriber("mirror_end_effector", mirror_end_effector, mirror_callback)
    
    def clean_shutdown():
        print("\nExiting example...")
        if not init_state:
            print("Disabling robot...")
            rs.disable()
    rospy.on_shutdown(clean_shutdown)

    rospy.spin()

if __name__ == "__main__":
    mirror_server()
