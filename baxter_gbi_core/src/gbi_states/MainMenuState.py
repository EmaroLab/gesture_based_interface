# -*- coding: latin-1 -*-
## @package MainMenuState
## This package describes the structure of main menu state 

from MenuState import MenuState

##  MainMenuState
#   inerithed form MenuState
class MainMenuState(MenuState):
    ## constructor
    #  @param trigger_event istance of FsmEvent class
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
    #  @param userdata 
    #  
    #  override of MenuState.update_variable_options
    #  updates the variable options of the menu
    def update_variable_options(self, userdata):
        return self.options

    ## method on_variable_selection
    #  @param userdata 
    #  @param index
    #  @param item 
    #
    #  override of MenuState.update_variable_options
    #  updates the variable options of the menu
    def on_variable_selection(self, index, item, userdata):
        return item

    ## method on_fixed_selection
    #  @param userdata 
    #  @param index
    #  @param item
    #  
    #  override of MenuState.on_fixed_selection
    #  in this menu this method should never be called 
    def on_fixed_selection(self, index, item, userdata):
        raise RuntimeError
