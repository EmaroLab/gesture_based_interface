#!/usr/bin/env python

"""
ROS node used to allow the user to control the baxter via mirroring.
"""

import argparse
import sys

import rospy

from BaxterGBI_pbr.msg import *


from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

from std_msgs.msg import Header


def mirror_callback(data):
    """
    Callback function associated with the topic 'mirror_end_effector'.
    Whenever a data is written in the topic, this function is called and obtain from ik_tracking function the joints values to assign and
    move the end effector to the goal.
    
    @type data.pose.pose.position: float[]
    @param data.pose.pose.position: position we want to achieve.
    @type data.pose.pose.orientation: float[]
    @param data.pose.pose.orientation: orientation we want to achieve (Quaternion).
    """
    
    global file_name
    
    rospy.loginfo("Reading...")
    file_name.write(str(data.pose.pose.position.x)+","+str(data.pose.pose.position.y)+","+str(data.pose.pose.position.z)+","+str(data.pose.pose.orientation.x)+","+str(data.pose.pose.orientation.y)+","+str(data.pose.pose.orientation.z)+","+str(data.pose.pose.orientation.w)+"\n")
    
       
  
def mirror_server():
    """
    Main of the node. Takes the information from the topic and move the baxter end effector based on those values.
    """
    
    rospy.loginfo("Initializing node... ")
    rospy.init_node('mirror_server')
    
    
    global file_name
    file_name = open("rosbag_output.txt","w+")
    
        
    rospy.Subscriber("odometry/baxter/right_hand", mirror_bag, mirror_callback)
    
    def clean_shutdown():
        rospy.loginfo("\nExiting example...")
        file_name.close()
    rospy.on_shutdown(clean_shutdown)

    rospy.spin()

if __name__ == "__main__":
    mirror_server()
