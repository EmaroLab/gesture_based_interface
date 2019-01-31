#!/usr/bin/env python

import argparse

import rospy
import sys

from BaxterGBI_pbr import JointRecorder

from baxter_interface import CHECK_VERSION
from BaxterGBI_pbr.msg import record_status, mirror_end_effector
from BaxterGBI_pbr.srv import CalibrateMirror, EnableMirroring


import tf

from math import sin

"""
Fake Client -> Placeholder for the real user. Just to test.

"""


#Mirror client -> publish data on a topic
def main():

    rospy.loginfo("Initializing node... ")
    rospy.init_node('mirror_client', anonymous=True)
    pub = rospy.Publisher('mirror_end_effector', mirror_end_effector, queue_size=2)
    
    init_pos = [0.163716,0.421201,-0.440442]
    init_orient = [0.54,-0.84,-0.005,0.003]
    
    rospy.wait_for_service('calibrate_mirroring')
    try:
        calibrate = rospy.ServiceProxy('calibrate_mirroring', CalibrateMirror)
        response = calibrate("left",init_pos,init_orient)
        if response.isError == 0:
            print("Calibrated!")
        else:
            print("Error calibrating")
            return
        
    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)
    
    rospy.wait_for_service('enable_mirroring')
    try:
        enableMirroring = rospy.ServiceProxy('enable_mirroring', EnableMirroring)
        response = enableMirroring(1)
        if response.isError == 0:
            print("Started!")
        else:
            print("Error starting mirroring")
            return
    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)
    
    
    
    rate = rospy.Rate(10)
    i = 0
    
    def clean_shutdown():
        rospy.loginfo("\nExiting...")
        rospy.loginfo("Stopping mirroring...")
            
        try:
            response = enableMirroring(0)
            if response.isError == 0:
                print("Mirroring has been stopped!")
            else:
                print("Error stopping mirroring")
                return
        except rospy.ServiceException, e:
            print("Service call failed: %s"%e)
    
    rospy.on_shutdown(clean_shutdown)

    
    while not rospy.is_shutdown():
        pos = []
        pos.append(init_pos[0]+sin(i))
        pos.append(init_pos[1])
        pos.append(init_pos[2]+sin(i))


        orient = []

        i += 0.2
        quaternion = tf.transformations.quaternion_from_euler(3.129668, -0.000121, -2.002885)
        #type(pose) = geometry_msgs.msg.Pose
        orient.append(quaternion[0])
        orient.append(quaternion[1])
        orient.append(quaternion[2])
        orient.append(quaternion[3])
        
        #print(str(orient))
        data = mirror_end_effector()
        data.position = pos
        data.quaternion = orient
        pub.publish(data)
        rate.sleep()
        
if __name__ == '__main__':
    main()
