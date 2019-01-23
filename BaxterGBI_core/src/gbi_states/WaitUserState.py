import rospy
from BlockingState import BlockingState

class WaitUserState(BlockingState):
    def __init__(self, trigger_event):
        outcomes=['reconf_requested',
                  'user_detected',
                  'preempted']

        BlockingState.__init__(self, outcomes, trigger_event)
        self.type = 'wait_user'

    def config(self, userdata):
        return 'reconf_requested'

    def user_detected(self, userdata):
        return 'user_detected'

    def publish_state(self):
        self.msg.context_type = self.type
        rospy.loginfo(self.msg)
        self.pub.publish(self.msg)