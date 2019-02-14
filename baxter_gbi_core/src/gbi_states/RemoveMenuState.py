## @package RemoveMenuState
## This package describes the structure
#  of the play menu state 

from MenuState import MenuState

##  RemoveMenuState
#   inerithed form MenuState
class RemoveMenuState(MenuState):
    ## the constructor
    #  @param trigger_event istance of the class FsmEvent
    def __init__(self, trigger_event):
        outcomes = ['back']

        MenuState.__init__(self,
                           outcomes,
                           trigger_event,
                           'Select the recording to delete',
                           input_keys=[],
                           fixed_options=['back'])

    ## method update_variable_options
    #  @param userdata 
    #
    #  override of MenuState.update_variable_options
    #  update the variable options of the menu
    def update_variable_options(self, userdata):
        return ['demo record']  # TODO: ask PBR the list of files

    ## method update_variable_options
    #  @param userdata 
    #  @param index
    #  @param item
    #  
    #  override of MenuState.on_variable_selection
    def on_variable_selection(self, index, item, userdata):
        # TODO: ask PBR to delete the record
        return 'back'