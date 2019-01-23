import rospy
from BlockingState import BlockingState

class ActionState(BlockingState):
    def __init__(self, outcomes, trigger_event, action, output_keys=[], input_keys=[]):
        BlockingState.__init__(self,
                               outcomes = ['done', 'user_missed', 'preempted'] + outcomes,
                               trigger_event = trigger_event,
                               output_keys= output_keys,
                               input_keys=input_keys)
        self.type = 'action'
        self.action = action

    def user_left(self, userdata):
        return 'user_missed'

    def publish_state(self):
        self.msg.context_type = self.type
        self.msg.pbr_action = self.action
        self.msg.pbr_msg = self.set_status()
        rospy.loginfo(self.msg)
        self.pub.publish(self.msg)

    def set_status(self):
        return ""