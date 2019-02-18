## @package SequenceMenuState
## This package describes the structure
#  of the sequence menu state 

from MenuState import MenuState

##  SequenceMenuState
#   inerithed form MenuState
class SequenceMenuState(MenuState):
    ## the constructor
    #  @param trigger_event istance of the class FsmEvent
    def __init__(self, trigger_event):
        outcomes=['play',
                  'back']

        MenuState.__init__(self,
                           outcomes,
                           trigger_event,
                           'Sequence menu',
                           input_keys=['sequence_idx', 'sequence_filename'])

        self.sequence = ["Add"]

    ## method update_variable_options
    #  @param userdata 
    #
    #  override of MenuState.update_variable_options
    #  update the variable options of the menu
    def update_variable_options(self, userdata):
        try:
            idx = userdata.sequence_idx
            try:
                fname = userdata.sequence_filename
                if not self.sequence[idx]:
                    self.sequence += "Add"
                self.sequence[idx] = fname
            except KeyError:
                if self.sequence[userdata.sequence_idx]:
                    del self.sequence[userdata.sequence_idx]
        except KeyError:
            pass
        return self.sequence

    ## method update_variable_options
    #  @param userdata 
    #  @param index
    #  @param item
    #  
    #  override of MenuState.on_variable_selection
    def on_variable_selection(self, index, item, userdata):
        userdata.selection = index
        return 'selection'

    # FIXME: 'play' outcome does not outputs sequence configuration