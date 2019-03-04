# -*- coding: latin-1 -*-
## @package WaitConfigState
## This package describes the structure of the 'wait for configuration' state

from BlockingState import BlockingState

## WaitConfigState
# inherited form BlockingState
class WaitConfigState(BlockingState):
    ## constructor
    #  @param trigger_event istance of FsmEvent class
    def __init__(self, trigger_event):
        outcomes=['config_available']
        BlockingState.__init__(self, outcomes, trigger_event)
        self.type = 'config_wait'

    ## method config
    # @param userdata
    #
    # callback of the trigger of a configuration event
    def config(self, userdata):
        return 'config_available'

    ## method publish_state
    #
    # publishes the message on the topic
    def publish_state(self):
        self.msg.context_type = self.type
        self.pub.publish(self.msg)
