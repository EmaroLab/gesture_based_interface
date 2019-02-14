## @package RecordMenuState
## This package describes the structure
#  of the record menu state 

from MenuState import MenuState

##  RecordMenuState
#   inerithed form MenuState
class RecordMenuState(MenuState):
    ## the constructor
    #  @param trigger_event istance of the class FsmEvent
    def __init__(self, trigger_event):
        outcomes = ['back',
                    'remove']

        MenuState.__init__(self,
                           outcomes,
                           trigger_event,
                           'Recording menu',
                           fixed_options=['back', 'remove', 'new'])

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
        if item == 'new':
            userdata.selection = "new_name"  # TODO: generate new unique name for recording
            return 'selection'
        else:
            return item