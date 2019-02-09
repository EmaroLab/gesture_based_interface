#!/usr/bin/env python

import argparse
import sys

import struct
import rospy

from baxter_interface import *

from baxter_gbi_pbr_srv.srv import *

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




## Class used to collapse in a single structure the joints value and a boolean for the error.
class ReturnValue(object):
    def __init__(self, limb_joints, isError):
        self.limb_joints = limb_joints
        self.isError = isError



## Method used to solve the Inverse Kinematic Problem and provide a solution, which is returned as output.
#
# @param limb: specify left or right limb.
# @param pos: position we want to achieve.
# @param orient: orientation we want to achieve (Quaternion).
#
# @returns ReturnValue object: limb joints position and 0 is ok, None and 1 if there is an error.
def ik_tracking(limb, pos, orient):

    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    poses = {
        limb: PoseStamped(
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

        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        rospy.loginfo("\nIK Joint Solution:\n"+ str(limb_joints))
        rospy.loginfo("------------------")
        rospy.loginfo("Response Message:\n"+ str(resp))

        return ReturnValue(limb_joints,0)
    else:
        rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")

    return ReturnValue(None,1)
