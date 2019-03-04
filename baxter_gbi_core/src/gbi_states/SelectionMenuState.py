# -*- coding: latin-1 -*-
## @package SubMacroMenuState
## This package describes the structure of the sequence menu state 

from FileMenuState import FileMenuState

## SubMacroMenuState
# inherited form FileMenuState
class SelectionMenuState(FileMenuState):
    #  constructor
    #  @param trigger_event instance of FsmEvent class
    def __init__(self, trigger_event):
        outcomes = ['back']
        FileMenuState.__init__(self,
                               outcomes,
                               trigger_event,
                               'Selection menu',
                               input_keys=['context_idx'],
                               output_keys=['context_idx_out'],
                               fixed_options=['back', 'clean'])

    # method update_variable_options
    # @param userdata
    #
    # override of MenuState.update_variable_options
    # update the variable options of the menu
    def execute(self, userdata):
        userdata.context_idx_out = userdata.context_idx
        return FileMenuState.execute(self, userdata)

    # method on_fixed_selection
    # @param userdata
    # @param index
    # @param item
    #  
    # override of MenuState.on_fixed_selection
    def on_fixed_selection(self, index, item, userdata):
        if item == 'clean':
            userdata.selected_item = 'Empty'
            return 'selection'
        return FileMenuState.on_fixed_selection(self, index, item, userdata)
