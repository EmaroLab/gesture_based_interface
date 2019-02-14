## @package WaitUserState
## This package describes the structure of the 'waiting for user' state

import rospy
from BlockingState import BlockingState
import time

##  WaitUserState
#   inerithed form BlockingState
class WaitUserState(BlockingState):
    ## the constructor
    #  @param outcomes outcomes of the state
    #  @param trigger_event istance of FsmEvent class
    def __init__(self, trigger_event):
        outcomes=['reconf_requested',
                  'user_detected']

        BlockingState.__init__(self, outcomes, trigger_event)
        self.type = 'wait_user'

    ## method config
    #  @param userdata 
    #
    #   callback of the trigger of a configuration event
    def config(self, userdata):
        return 'reconf_requested'

    ## method user_detected
    #  @param userdata 
    #
    #   callback of the trigger for user presence
    def user_detected(self, userdata):
        return 'user_detected'

    ## method publish_state
    #  publish the message on the topic
    def publish_state(self):
        self.msg.context_type = self.type
        rospy.loginfo(self.msg)
        self.pub.publish(self.msg)
