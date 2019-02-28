#!/usr/bin/env python

import os
import argparse
import sys
import actionlib

import rospy

import baxter_interface

from baxter_interface import CHECK_VERSION
from baxter_gbi_pbr_srvs.srv import *
from baxter_gbi_pbr_msgs.msg import *
from pbr_header import *



class PlaybackObj(object):

    def __init__(self):
        self.pause_state = 0
        self.paused_time = 0
        self.line_executed = 0
        self.stop = 0

    def try_float(self, x):
        try:
            return float(x)
        except ValueError:
            return None


    ## Cleans a single line of recorded joint positions
    #
    # @param line: the line described in a list to process
    # @param names: joint name keys
    def clean_line(self, line, names):
        #convert the line of strings to a float or None
        line = [self.try_float(x) for x in line.rstrip().split(',')]
        #zip the values with the joint names
        combined = zip(names[1:], line[1:])
        #take out any tuples that have a none value
        cleaned = [x for x in combined if x[1] is not None]
        #convert it to a dictionary with only valid commands
        command = dict(cleaned)
        left_command = dict((key, command[key]) for key in command.keys()
                            if key[:-2] == 'left_')
        right_command = dict((key, command[key]) for key in command.keys()
                             if key[:-2] == 'right_')
        return (command, left_command, right_command, line)


    ## Loops through csv file
    #
    # @param filename: the file to play
    # @param loops: number of times to loop
    #              values < 0 mean 'infinite'
    # @param scale_vel: number to scale the velocity of the playback
    #
    # Does not loop indefinitely, but only until the file is read
    # and processed. Reads each line, split up in columns and
    # formats each line into a controller command in the form of
    # name/value pairs. Names come from the column headers
	
    def map_file(self, filename, loops=1, scale_vel=100, service=None):
        left = baxter_interface.Limb('left')
        right = baxter_interface.Limb('right')
        grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
        grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
        rate = rospy.Rate(1000)

        if grip_left.error():
            grip_left.reset()
        if grip_right.error():
            grip_right.reset()
        if (not grip_left.calibrated() and
            grip_left.type() != 'custom'):
            grip_left.calibrate()
        if (not grip_right.calibrated() and
            grip_right.type() != 'custom'):
            grip_right.calibrate()

        rospy.loginfo("Playing back: %s" % (filename,))
        with open(filename, 'r') as f:
            lines = f.readlines()
        keys = lines[0].rstrip().split(',')

        l = 0

        self.pause_state = 0
        self.paused_time = 0
        self.stop = 0
        # If specified, repeat the file playback 'loops' number of times
        while loops < 1 or l < loops:
            if service and service.is_preempt_requested(): break
            i = 0
            l += 1
            rospy.loginfo("Moving to start position...")
            feedback = playbackFeedback()
            feedback.percent_complete = -1
            service.publish_feedback(feedback)
            _cmd, lcmd_start, rcmd_start, _raw = self.clean_line(lines[1], keys)
            left.move_to_joint_positions(lcmd_start, threshold=baxter_interface.settings.JOINT_ANGLE_TOLERANCE*2.5)
            right.move_to_joint_positions(rcmd_start, threshold=baxter_interface.settings.JOINT_ANGLE_TOLERANCE*2.5)
            start_time = rospy.get_time()
            
            number_lines = 0
            
            while ((number_lines < len(lines)-1) and self.stop == 0):
                if service and service.is_preempt_requested(): break
                feedback.percent_complete = (number_lines/float((len(lines)-1))*100.0)/float(loops - l + 1)
                #rospy.loginfo("Complete:"+str(feedback.percent_complete))
                rospy.loginfo("Preempted :"+str(service.is_preempt_requested()))
                service.publish_feedback(feedback)

                if self.pause_state == 0:
                    number_lines += 1
                    
                    values = lines[(number_lines)]
                 
                    i += 1
                    loopstr = str(loops) if loops > 0 else "forever"
                    sys.stdout.write("\r Record %d of %d, loop %d of %s" %
                                     (i, len(lines)-1, l, loopstr))
                    sys.stdout.flush()

                    cmd, lcmd, rcmd, values = self.clean_line(values, keys)
                    #command this set of commands until the next frame
                    while (rospy.get_time() - start_time - self.paused_time) < values[0]*(100.0/scale_vel):
                        if service and service.is_preempt_requested(): break
                        if rospy.is_shutdown():
                            rospy.loginfo("\n Aborting - ROS shutdown")
                            return False
                        if len(lcmd):
                            left.set_joint_positions(lcmd)
                        if len(rcmd):
                            right.set_joint_positions(rcmd)
                        if ('left_gripper' in cmd and
                            grip_left.type() != 'custom'):
                            grip_left.command_position(cmd['left_gripper'])
                        if ('right_gripper' in cmd and
                            grip_right.type() != 'custom'):
                            grip_right.command_position(cmd['right_gripper'])
                        rate.sleep()
                    rospy.loginfo("-- "+str(rospy.get_time()))
                    
                    self.line_executed = number_lines                   
 
        rospy.loginfo("Moving to start position...")

        feedback.percent_complete = -1
        service.publish_feedback(feedback)
        _cmd, lcmd_start, rcmd_start, _raw = self.clean_line(lines[len(lines)-1], keys)
        left.move_to_joint_positions(lcmd_start)
        right.move_to_joint_positions(rcmd_start)
        feedback.percent_complete = 100
        service.publish_feedback(feedback)
        rospy.loginfo("End Playback!")
        return True
