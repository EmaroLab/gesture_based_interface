# -*- coding: latin-1 -*-
## @package MainMenuState
## This package describes the structure of the main menu state

from MenuState import MenuState

## MainMenuState
# inherited form MenuState
class MainMenuState(MenuState):
    ## constructor
    # @param trigger_event istance of FsmEvent class
    def __init__(self, trigger_event):
        self.options = ['play',
                        'record',
                        'macro',
                        'sequence']

        MenuState.__init__(self,
                           outcomes=self.options,
                           trigger_event=trigger_event,
                           page_title='Main menu',
                           fixed_options=[])

    ## method update_variable_options
    # override of MenuState.update_variable_options
    # updates the variable options of the menu
    #  
    # @param userdata
    def update_variable_options(self, userdata):
        return self.options

    ## method on_variable_selection
    # override of MenuState.update_variable_options
    # updates the variable options of the menu
    #
    # @param userdata
    # @param index
    # @param item
    def on_variable_selection(self, index, item, userdata):
        return item

    ## method on_fixed_selection
    # override of MenuState.on_fixed_selection
    # in this menu this method should never be called
    #  
    # @param userdata
    # @param index
    # @param item
    def on_fixed_selection(self, index, item, userdata):
        raise RuntimeError
