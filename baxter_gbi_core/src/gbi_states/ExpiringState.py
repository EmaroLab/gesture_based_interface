import rospy
import time
from threading import Timer
from BlockingState import BlockingState

class ExpiringState(BlockingState):
    ## constructor
    # @param outcomes possible outcomes of the state
    # @param trigger_event object belonging to FsmEvent class
    # @param action executable action
    # @param output_keys set of output data
    # @param input_keys set of input data
    def __init__(self, outcomes, trigger_event, output_keys=[], input_keys=[]):
        BlockingState.__init__(self,
                               outcomes = ['user_missed'] + outcomes,
                               trigger_event = trigger_event,
                               output_keys= output_keys,
                               input_keys=input_keys)
        ## timeout for the user presence
        self.timeout = 5
        self.t = None

    def user_left(self, userdata):
        return 'user_missed'

    ## method user_dected
    #  override of BlockingState.user_detected
    #  @param userdata data in input to the state
    def user_detected(self, userdata):
        if self.t:
            self.t.cancel()
        self.t = Timer(self.timeout, self.timeout_cb)
        self.t.start()
        return None

    def timeout_cb(self):
        self._trigger_event.signal('user_left')

    def execute(self, userdata):
        self.t = Timer(self.timeout, self.timeout_cb)
        self.t.start()
        ret = BlockingState.execute(self, userdata)
        self.t.cancel()
        return ret
