## @package ActionState
#  This package define the essential structure
#  for all the action states

import rospy
from ExpiringState import ExpiringState
import time

##  ActionState
#   inerithed form BlockingState 
class ActionState(ExpiringState):
    ## the constructor
    # @param outcomes possible outcomes of the state
    # @param trigger_event object of the class FsmEvent
    # @param action executable action
    # @param output_keys set of the data in output
    # @param input_keys set of the data in input
    def __init__(self, outcomes, trigger_event, action, output_keys=[], input_keys=[]):
        ExpiringState.__init__(self,
                               outcomes = ['done'] + outcomes,
                               trigger_event = trigger_event,
                               output_keys= output_keys,
                               input_keys=input_keys)
        ## attribute of type
        self.type = 'action'
        ## attribute of type of action
        self.action = action
        ## attribute for the timeout

    ## method publish_state
    #  overide of BlockingState.publish_state
    def publish_state(self):
        self.msg.context_type = self.type
        self.msg.pbr_action = self.action
        self.msg.pbr_msg = self.set_status()
        rospy.loginfo(self.msg)
        self.pub.publish(self.msg)

    ## method set_status
    def set_status(self):
        return ""