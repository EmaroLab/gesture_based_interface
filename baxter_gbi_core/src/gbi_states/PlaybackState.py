# -*- coding: latin-1 -*-
## @package ActionState
#  This package defines the essential structure for all the action states

import rospy
from ActionState import ActionState
import rospy
import actionlib
from baxter_gbi_pbr_msgs.msg import playbackAction, playbackGoal
from baxter_gbi_pbr_srvs.srv import *

import os
debug = os.environ.get('BGI_DEBUG')

##  ActionState

class PlaybackState(ActionState):
    ## the constructor
    # @param outcomes possible outcomes of the state
    # @param trigger_event object of the class FsmEvent
    # @param action executable action
    # @param output_keys set of the data in output
    # @param input_keys set of the data in input
    end = False
    killing = False

    def __init__(self, outcomes, trigger_event, status, output_keys=[], input_keys=[]):
        ActionState.__init__(self,
                             outcomes=["stop"]+outcomes,
                             status=status,
                             trigger_event=trigger_event,
                             output_keys=output_keys,
                             input_keys=input_keys)

        self.goal = playbackGoal()
        if not debug:
            rospy.wait_for_service('/pause_resume')
        self.pause_resume = rospy.ServiceProxy('/pause_resume', PauseResume)
        self.playback = actionlib.SimpleActionClient('/playback', playbackAction)
        if not debug:
            self.playback.wait_for_server()

    def action_5(self, userdata):
        PlaybackState.killing = True
        self.playback.cancel_all_goals()
        while not PlaybackState.end:
            pass
        return 'stop'
    
    def user_left(self, userdata):
        PlaybackState.killing = True
        self.playback.cancel_all_goals()
        while not PlaybackState.end:
            pass
        return 'user_missed'
