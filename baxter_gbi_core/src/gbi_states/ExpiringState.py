# -*- coding: latin-1 -*-
## @package ExpiringState
# The package defines the structure of the expiring state

from threading import Timer
from BlockingState import BlockingState

## ExpiringState
# inherited from BlockingState
class ExpiringState(BlockingState):
    ## constructor
    # @param outcomes possible outcomes of the state
    # @param trigger_event object of the class FsmEvent
    # @param action executable action
    # @param output_keys set of the data in output
    # @param input_keys set of the data in input
    def __init__(self, outcomes, trigger_event, output_keys=[], input_keys=[]):
        BlockingState.__init__(self,
                               outcomes = ['user_missed'] + outcomes,
                               trigger_event = trigger_event,
                               output_keys= output_keys,
                               input_keys=input_keys)
        ## timeout for the user presence
        self.timeout = 1
        self.t = None

    ## method user_left
    # @param userdata
    #
    # callback of the trigger "user_left"
    def user_left(self, userdata):
        return 'user_missed'

    ## method user_dected
    # @param userdata data in input to the state
    #
    # override of BlockingState.user_detected
    def user_detected(self, userdata):
        if self.t:
            self.t.cancel()
        self.t = Timer(self.timeout, self.timeout_cb)
        self.t.start()
        return None

    ## method timeout_cb
    #
    # timeout when the user leaves
    def timeout_cb(self):
        self.signal('user_left')

    ## method execute
    # @param userdata
    #
    # starts the timer and executes
    def execute(self, userdata):
        self.t = Timer(self.timeout, self.timeout_cb)
        self.t.start()
        ret = BlockingState.execute(self, userdata)
        self.t.cancel()
        return ret
