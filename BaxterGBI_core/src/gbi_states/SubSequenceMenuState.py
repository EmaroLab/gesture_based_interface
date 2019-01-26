## @package SubSequenceMenuState
## This package describes the structure
#  of the sequence menu state 
from MenuState import MenuState

##  SubSequenceMenuState
#   inerithed form MenuState
class SubSequenceMenuState(MenuState):
    ## the constructor
    #  @param trigger_event istance of the class FsmEvent
    def __init__(self, trigger_event):
        outcomes = ['back']

        MenuState.__init__(self,
                           outcomes,
                           trigger_event,
                           'Sequence menu - selection',
                           input_keys=['sequence_idx'],
                           fixed_options=['back', 'clean'])

    ## method update_variable_options
    #  @param userdata 
    #
    #  override of MenuState.update_variable_options
    #  update the variable options of the menu
    def update_variable_options(self, userdata):
        return ['demo record']  # TODO: ask PBR the list of files

    ## method on_fixed_selection
    #  @param userdata 
    #  @param index
    #  @param item
    #  
    #  override of MenuState.on_fixed_selection
    def on_fixed_selection(self, index, item, userdata):
        if item == 'clean':
            userdata.selection = ''
            return 'selection'
        else:
            return item