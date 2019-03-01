# -*- coding: latin-1 -*-
## @package WaitUserState
## This package describes the structure of the 'waiting for user' state

from BlockingState import BlockingState

## WaitUserState
# inherited form BlockingState
class WaitUserState(BlockingState):
    ## constructor
    # @param outcomes outcomes of the state
    # @param trigger_event instance of FsmEvent class
    def __init__(self, trigger_event):
        outcomes=['reconf_requested',
                  'user_detected']

        BlockingState.__init__(self, outcomes, trigger_event)
        self.type = 'wait_user'

    ## method config
    # @param userdata
    #
    # callback of the trigger of a
    # configuration event
    def config(self, userdata):
        return 'reconf_requested'

    ## method user_detected
    # @param userdata
    #
    # callback of the trigger for user presence
    def user_detected(self, userdata):
        return 'user_detected'

    ## method publish_state
    #
    # publishes the message on the topic
    def publish_state(self):
        self.msg.context_type = self.type
        self.pub.publish(self.msg)
