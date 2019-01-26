## @package MenuState
## This package describes the general structure
#  of the menu states 

import rospy
from BlockingState import BlockingState
import time

##  MenuState
#   inerithed form BlockingState
class MenuState(BlockingState):
    ## the constructor
    #  @param outcomes outcomes of the state
    #  @param trigger_event istance of the class FsmEvent
    #  @param page_title name of the gui menu page
    #  @param output_keys set of data in the output state
    #  @param input_keys set of data in the input state
    #  @param fixed_options fixed options of the menu
    def __init__(self, outcomes, trigger_event, page_title, output_keys=[], input_keys=[], fixed_options=['back', 'play']):
        BlockingState.__init__(self,
                               outcomes = ['user_missed', 'selection'] + outcomes,
                               trigger_event = trigger_event,
                               output_keys=['selection'] + output_keys,
                               input_keys = input_keys)

        ## type for the topic
        self.type = 'menu'
        self.title = page_title
        self.variable_options = []
        self.fixed_options = fixed_options
        self.selection = 0
        ## timeout_t for the MenuState
        self.timeout_t=time.time()+self.timeout

    ## method action_1
    #  @param userdata 
    #  
    #  override of BlockingState.action_1
    #  action_1 assuming to go up in menu
    def action_1(self, userdata):
        max_selection = len(self.variable_options) + len(self.fixed_options)
        if self.selection < max_selection:
            self.selection += 1
        return None

    ## method action_2
    #  @param userdata 
    #  
    #  override of BlockingState.action_2
    #  action_2 assuming to go down in menu
    def action_2(self, userdata):
        if self.selection > 0:
            self.selection -= 1
        return None

    ## method action_3
    #  @param userdata 
    #  
    #  override of BlockingState.action_3
    #  assuming that the action_3 is the selection
    def action_3(self, userdata):
        if self.selection < len(self.variable_options):
            item = self.variable_options[self.selection]
            return self.on_variable_selection(self.selection, item, userdata)
        else:
            item = self.fixed_options[self.selection - len(self.variable_options)]
            return self.on_fixed_selection(self.selection - len(self.variable_options), item, userdata)

    ## method user_left
    #  @param userdata 
    #  
    #  override of BlockingState.user_left
    #  return in putcome user_missed
    def user_left(self, userdata):
        return 'user_missed'

    ## method user_detected
    #  @param userdata 
    #  
    #  override of BlockingState.action_1
    #  reset the timeout_t
    def user_detected(self, userdata):
        self.timeout_t=time.time()+self.timeout
        return None

    ## method execute
    #  @param userdata 
    #  
    #  override of BlockingState.execute
    def execute(self, userdata):
        self.variable_options = self.update_variable_options(userdata)
        return BlockingState.execute(self, userdata)

    ## method publish_state
    #  
    #  override of BlockingState.publish_state
    #  publish the message on the topic
    def publish_state(self):
        self.msg.context_type = self.type
        self.msg.m_title = self.title
        self.msg.m_options = self.variable_options
        self.msg.m_fixed_options = self.fixed_options
        self.msg.m_selection = self.selection
        rospy.loginfo(self.msg)
        self.pub.publish(self.msg)

    ## method update_variable_options
    #  @param userdata 
    #  
    #  prototype of update_variable_options 
    def update_variable_options(self, userdata):
        return []

    ## method update_variable_options
    #  @param userdata 
    #  @param index
    #  @param item
    #  
    #  fill the field userdata.selection with item 
    def on_variable_selection(self, index, item, userdata):
        userdata.selection = index
        return 'selection'

    ## method on_fixed_selection
    #  @param userdata 
    #  @param index
    #  @param item
    #  
    #  return item
    def on_fixed_selection(self, index, item, userdata):
        return item