# -*- coding: latin-1 -*-
## @package ActionState
# This package defines the essential structure for all the action states

from ActionState import ActionState
import rospy
import actionlib
from baxter_gbi_pbr_msgs.msg import playbackAction, playbackGoal
from baxter_gbi_pbr_srvs.srv import *

import os
debug = os.environ.get('BGI_DEBUG')

## PlaybackState
#  inherited form ActionState
class PlaybackState(ActionState):
    ## constructor
    # @param outcomes possible outcomes of the state
    # @param trigger_event object of the class FsmEvent
    # @param action executable action
    # @param output_keys set of the data in output
    # @param input_keys set of the data in input
    end = False
    killing = False

    def __init__(self, outcomes, trigger_event, status, output_keys=[], input_keys=[], fixed_options = []):
        ActionState.__init__(self,
                             outcomes=["stop"]+outcomes,
                             status=status,
                             trigger_event=trigger_event,
                             output_keys=output_keys,
                             input_keys=input_keys,
                             fixed_options = fixed_options)

        self.goal = playbackGoal()
        if not debug:
            rospy.wait_for_service('/pause_resume')
        self.pause_resume = rospy.ServiceProxy('/pause_resume', PauseResume)
        self.playback = actionlib.SimpleActionClient('/playback', playbackAction)
        if not debug:
            self.playback.wait_for_server()

    ## method action_5
    # override of BlockingState.action_5
    # where action_5 is assumed to be "exit"
    #
    # @param userdata
    def action_1(self, userdata):
        PlaybackState.killing = True
        self.playback.cancel_all_goals()
        while not PlaybackState.end:
            pass
        return 'stop'

    ## method user_left
    # called when the user leaves
    #
    # @param userdata
    def user_left(self, userdata):
        PlaybackState.killing = True
        self.playback.cancel_all_goals()
        while not PlaybackState.end:
            pass
        return 'user_missed'
