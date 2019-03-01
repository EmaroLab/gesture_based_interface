# -*- coding: latin-1 -*-
from ActionState import ActionState
import rospy
import actionlib
from baxter_gbi_pbr_msgs.msg import playbackAction, playbackGoal
from baxter_gbi_pbr_srvs.srv import *


class SequenceState(ActionState):
    def __init__(self, trigger_event):
        input_keys = ['sequence', 'new_sequence']
        output_keys = ['filename', 'new_sequence']

        ActionState.__init__(self,
                             outcomes=['play'],
                             trigger_event=trigger_event,
                             status='wait',
                             output_keys=output_keys,
                             input_keys=input_keys)
        self.index = -1
        self.sequence = []

    @staticmethod
    def play_file(userdata, index):
        userdata.filename = userdata.sequence[index]
        return "play"

    def new_sequence(self, userdata):
        try:
            new = userdata.new_sequence
            userdata.new_sequence = False
            print "New sequence "
            print new
            return new
        except KeyError:
            return False

    def execute(self, userdata):
        if self.new_sequence(userdata):
            self.index = -1
        if self.index < len(userdata.sequence)-1:
            self.index += 1
            return self.play_file(userdata, self.index)
        else:
            return "done"
