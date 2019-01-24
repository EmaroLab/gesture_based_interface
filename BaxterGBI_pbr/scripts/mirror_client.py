#!/usr/bin/env python

import argparse

import rospy
import sys

import baxter_interface
from BaxterGBI_pbr import JointRecorder

from baxter_interface import CHECK_VERSION
from BaxterGBI_pbr.msg import record_status, mirror_end_effector


import tf

"""

"""

    

#Mirror client -> publish data on a topic
def main():

    print("Initializing node... ")
    rospy.init_node('mirror_client', anonymous=True)
    pub = rospy.Publisher('mirror_end_effector', mirror_end_effector, queue_size=2)

    #TODO -> open file and change the hz
    
    rate = rospy.Rate(1) # 10hz
    state = 0
    i = 0
    while not rospy.is_shutdown():
        #TODO -> set up the message and publish it
        pos = []
        pos.append(0.163716+i/5)
        pos.append(0.421201+i/4)
        pos.append(-0.440442)


        orient = []
        i += 0.2
        quaternion = tf.transformations.quaternion_from_euler(3.129668, -0.000121-i, -2.002885+2*i)
        #type(pose) = geometry_msgs.msg.Pose
        orient.append(quaternion[0])
        orient.append(quaternion[1])
        orient.append(quaternion[2])
        orient.append(quaternion[3])
        
        data = mirror_end_effector()
        data.position = pos
        data.quaternion = orient
        pub.publish(data)
        rate.sleep()


if __name__ == '__main__':
    main()
