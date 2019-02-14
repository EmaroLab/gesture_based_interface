## @package WaitConfigState
## This package describes the structure
#  of the state waiting for the configuration

import rospy
from BlockingState import BlockingState

##  WaitConfigState
#   inerithed form BlockingState
class WaitConfigState(BlockingState):
    ## the constructor
    #  @param trigger_event istance of the class FsmEvent
    def __init__(self, trigger_event):
        outcomes=['config_available']

        BlockingState.__init__(self, outcomes, trigger_event)
        self.type = 'config_wait'

    ## method config
    #  @param userdata 
    #
    #   call back of the trigger of a configuration event
    def config(self, userdata):
        return 'config_available'

    ## method publish_state
    #  publish the message on the topic
    def publish_state(self):
        self.msg.context_type = self.type
        rospy.loginfo(self.msg)
        self.pub.publish(self.msg)