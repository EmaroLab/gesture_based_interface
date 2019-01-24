import rospy
from BlockingState import BlockingState


class WaitConfigState(BlockingState):
    def __init__(self, trigger_event):
        outcomes=['config_available']

        BlockingState.__init__(self, outcomes, trigger_event)
        self.type = 'config_wait'

    def config(self, userdata):
        return 'config_available'

    def publish_state(self):
        self.msg.context_type = self.type
        rospy.loginfo(self.msg)
        self.pub.publish(self.msg)