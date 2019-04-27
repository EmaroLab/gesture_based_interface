# -*- coding: latin-1 -*-
## @package ActionState
#  This package defines the essential structure for all the action states

from ExpiringState import ExpiringState
from baxter_gbi_pbr_srvs.srv import *

## ActionState
# inherited from ExpiringStates
class ActionState(ExpiringState):
    ## constructor
    # @param outcomes possible outcomes of the state
    # @param trigger_event object of the class FsmEvent
    # @param action executable action
    # @param output_keys set of the data in output
    # @param input_keys set of the data in input
    def __init__(self, outcomes, trigger_event, status, output_keys=[], input_keys=[], fixed_options = []):
        ExpiringState.__init__(self,
                               outcomes = ['done'] + outcomes,
                               trigger_event=trigger_event,
                               output_keys=output_keys,
                               input_keys=input_keys)

        ## attribute of type action
        self.fixed_options = ['stop'] + fixed_options
        self.type = 'action'
        self.status = status

    ## method action_5
    # override of BlockingState.action_1
    # where action_5 is assumed to be "done"
    #
    # @param userdata
    def action_1(self, userdata):
        return 'done'

    ## method done
    # action terminated
    #
    # @param userdata
    def done(self, userdata):
        print("DONE")
        return 'done'

    def numeric_input(self, number, userdata):
        f = getattr(self, "action_%d" % (number+1), None)
        if f is not None:
            return f(userdata)

    ## method publish_state
    #
    # override of BlockingState.publish_state
    def publish_state(self):
        self.msg.context_type = self.type
        self.msg.pbr_action = self.status
        self.msg.pbr_msg = self.set_status()
        self.msg.action_options = self.fixed_options
        self.pub.publish(self.msg)

    ## method set_status
    #
    # returns an empty string
    def set_status(self):
        return ""
