# -*- coding: latin-1 -*-
from ActionState import ActionState
import rospy
import actionlib
from baxter_gbi_pbr_msgs.msg import playbackAction, playbackGoal
from baxter_gbi_pbr_srvs.srv import *


class SequenceState(ActionState):
    def __init__(self, trigger_event):
        input_keys = ['sequence']
        output_keys = ['filename']

        ActionState.__init__(self,
                             outcomes=['play'],
                             trigger_event=trigger_event,
                             status='wait',
                             output_keys=output_keys,
                             input_keys=input_keys)
        self.index = 0
        self.sequence = []

    @staticmethod
    def play_file(userdata, index):
        userdata.filename = userdata.filenames[index]
        return "play"

    def fun(self,sequence,vel):
        self.playback.cancel_goal()
        self.goal.msg.filename=sequence
        self.goal.msg.loops=1
        self.goal.msg.scale_vel=vel
        self.playback.send_goal(self.goal,self.cb_done,None,self.feedback_cb)

    def execute(self, userdata):
        '''
        try:
            self.sequence = userdata.sequence
        except KeyError:
            pass
        '''

        if self.index < len(userdata.sequence)-1:
            self.index += 1
            return self.play_file(userdata, self.index)
        else:
            return "done"
