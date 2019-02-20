## @package MacroMenuState
## This package describes the general structure
#  of the macro menu state
from MenuState import MenuState

##  MacroMenuState
#   inerithed form BlockingState
class MacroMenuState(MenuState):
    ## the constructor
    #  @param trigger_event istance of the class FsmEvent
    def __init__(self, trigger_event):

        outcomes = ['play','back']

        MenuState.__init__(self,
                           outcomes,
                           trigger_event,
                           'Macro menu',
                           input_keys=['macro_idx', 'macro_filename'])

        self.macro_slots = ["Empty", "Empty", "Empty", "Empty"]

    ## method update_variable_options
    #  @param userdata 
    #  
    #  override of MenuState.update_variable_options
    #  update the variable options of the menu
    def update_variable_options(self, userdata):
        try:
            self.macro_slots[userdata.macro_idx] = userdata.macro_filename
        except KeyError:
            pass
        return self.macro_slots


    ## method on_variable_selection
    #  @param userdata 
    #  @param index
    #  @param item 
    #
    #  override of MenuState.update_variable_options
    #  update the variable options of the menu
    def on_variable_selection(self, index, item, userdata):
        userdata.selection = index
        return 'selection'


    # FIXME: 'play' outcome does not outputs macro configuration