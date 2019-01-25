import rospy
from BlockingState import BlockingState
import time

class MenuState(BlockingState):
    def __init__(self, outcomes, trigger_event, page_title, output_keys=[], input_keys=[], fixed_options=['back', 'run']):
        BlockingState.__init__(self,
                               outcomes = ['user_missed', 'selection'] + outcomes,
                               trigger_event = trigger_event,
                               output_keys=['selection'] + output_keys,
                               input_keys = input_keys)

        self.type = 'menu'
        self.title = page_title
        self.variable_options = []
        self.fixed_options = fixed_options
        self.selection = 0
        self.timeout_t=time.time()+self.timeout

    def action_1(self, userdata):
        max_selection = len(self.variable_options) + len(self.fixed_options)
        if self.selection < max_selection:
            self.selection += 1
        return None

    def action_2(self, userdata):
        if self.selection > 0:
            self.selection -= 1
        return None

    def action_3(self, userdata):
        if self.selection < len(self.variable_options):
            item = self.variable_options[self.selection]
            return self.on_variable_selection(self.selection, item, userdata)
        else:
            item = self.fixed_options[self.selection - len(self.variable_options)]
            return self.on_fixed_selection(self.selection - len(self.variable_options), item, userdata)

    def user_left(self, userdata):
        return 'user_missed'

    def user_detected(self, userdata):
        self.timeout_t=time.time()+self.timeout
        return None

    def execute(self, userdata):
        self.variable_options = self.update_variable_options(userdata)
        return BlockingState.execute(self, userdata)

    def publish_state(self):
        self.msg.context_type = self.type
        self.msg.m_title = self.title
        self.msg.m_options = self.variable_options
        self.msg.m_fixed_options = self.fixed_options
        self.msg.m_selection = self.selection
        rospy.loginfo(self.msg)
        self.pub.publish(self.msg)

    def update_variable_options(self, userdata):
        return []

    def on_variable_selection(self, index, item, userdata):
        userdata.selection = item
        return 'selection'

    def on_fixed_selection(self, index, item, userdata):
        return item