# -*- coding: latin-1 -*-
## @package ActionState
#  This package defines the essential structure for all the action states

import rospy
from ExpiringState import ExpiringState
import rospy
import actionlib
from baxter_gbi_pbr_msgs.msg import playbackAction, playbackGoal
from baxter_gbi_pbr_srvs.srv import *
##  ActionState

class ActionState(ExpiringState):
    ## the constructor
    # @param outcomes possible outcomes of the state
    # @param trigger_event object of the class FsmEvent
    # @param action executable action
    # @param output_keys set of the data in output
    # @param input_keys set of the data in input
    progress = 0
    end = False
    killing = False

    def __init__(self, outcomes, trigger_event, status, output_keys=[], input_keys=[]):
        ExpiringState.__init__(self,
                               outcomes = ['done'] + outcomes,
                               trigger_event = trigger_event,
                               output_keys= output_keys,
                               input_keys=input_keys)
        ## attribute of type action
        self.type = 'action'
        self.status = status
        self.goal = playbackGoal()
        rospy.wait_for_service('pause_resume')
        self.pause_resume = rospy.ServiceProxy('pause_resume', PauseResume)
        self.playback = actionlib.SimpleActionClient('playback', playbackAction)
        self.playback.wait_for_server()

    def action_5(self, userdata):
        ActionState.killing = True
        self.playback.cancel_all_goals()
        while not ActionState.end:
            pass
        return 'done'
    
    def user_left(self, userdata):
        ActionState.killing = True
        self.playback.cancel_all_goals()
        while not ActionState.end:
            pass
        return 'user_missed'
    
    def done(self, userdata):
        return 'done'

    ## method publish_state
    #  overide of BlockingState.publish_state
    def publish_state(self):
        self.msg.context_type = self.type
        self.msg.pbr_action = self.status
        self.msg.pbr_msg = self.set_status()
        #rospy.loginfo(self.msg)
        self.pub.publish(self.msg)

    ## method set_status
    def set_status(self):
        return ""
