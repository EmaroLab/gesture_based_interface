# -*- coding: latin-1 -*-
## @package MenuState
## This package describes the general structure of the menu states

from ExpiringState import ExpiringState

## MenuState
# inherited form BlockingState
class MenuState(ExpiringState):
    ## constructor
    # @param outcomes outcomes of the state
    # @param trigger_event instance of the class FsmEvent
    # @param page_title name of the gui menu page
    # @param output_keys set of data in the output state
    # @param input_keys set of data in the input state
    # @param fixed_options fixed options of the menu
    def __init__(self, outcomes, trigger_event, page_title, output_keys=[], input_keys=[], fixed_options=[]):
        ExpiringState.__init__(self,
                               outcomes = ['selection'] + outcomes,
                               trigger_event = trigger_event,
                               output_keys=['selected_item', 'selected_idx'] + output_keys,
                               input_keys = input_keys)

        ## type for the topic
        self.type = 'menu'
        self.title = page_title
        self.variable_options = []
        self.fixed_options = fixed_options
        self.selection = 0

    ## method action_1
    # override of BlockingState.action_1 where
    # action_1 is assumed to be "go up in menu"
    #  
    # @param userdata
    def action_1(self, userdata):
        max_selection = len(self.variable_options) + len(self.fixed_options)
        if self.selection < max_selection - 1:
            self.selection += 1
        return None

    ## method action_2
    # override of BlockingState.action_2 where
    # action_2 is assume to be "go down in menu"
    #  
    # @param userdata
    def action_2(self, userdata):
        if self.selection > 0:
            self.selection -= 1
        return None

    ## method action_3
    # override of BlockingState.action_3
    # assuming that the action_3 is the selection
    #  
    # @param userdata
    def action_3(self, userdata):
        if self.selection < len(self.variable_options):
            item = self.variable_options[self.selection]
            return self.on_variable_selection(self.selection, item, userdata)
        else:
            item = self.fixed_options[self.selection - len(self.variable_options)]
            return self.on_fixed_selection(self.selection - len(self.variable_options), item, userdata)

    ## method execute
    # override of BlockingState.execute
    #  
    # @param userdata
    def execute(self, userdata):
        self.selection=0
        self.variable_options = self.update_variable_options(userdata)
        return ExpiringState.execute(self, userdata)

    ## method publish_state
    #  
    # override of BlockingState.publish_state
    # publish the message on the topic
    def publish_state(self):
        self.msg.context_type = self.type
        self.msg.m_title = self.title
        self.msg.m_options = self.variable_options
        self.msg.m_fixed_options = self.fixed_options
        self.msg.m_selection = self.selection
        #rospy.loginfo(self.msg)
        self.pub.publish(self.msg)

    ## method update_variable_options
    # prototype of update_variable_options
    #  
    # @param userdata
    def update_variable_options(self, userdata):
        return []

    ## method update_variable_options
    # fills the field userdata.selection with item
    #  
    # @param userdata
    # @param index
    # @param item
    def on_variable_selection(self, index, item, userdata):
        userdata.selected_idx = index
        userdata.selected_item = item
        return 'selection'

    ## method on_fixed_selection
    # returns item
    #  
    # @param userdata
    # @param index
    # @param item
    def on_fixed_selection(self, index, item, userdata):
        return item
