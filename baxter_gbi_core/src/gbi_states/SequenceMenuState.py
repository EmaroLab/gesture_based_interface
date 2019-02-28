# -*- coding: latin-1 -*-
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
        outcomes = ['play',
                    'back']

        MenuState.__init__(self,
                           outcomes=outcomes,
                           trigger_event=trigger_event,
                           page_title='Sequence menu',
                           fixed_options=outcomes,
                           input_keys=['sequence_idx', 'sequence_item'],
                           output_keys=['sequence'])

        self.sequence = ["Add"]

    ## method update_variable_options
    #  @param userdata 
    #
    #  override of MenuState.update_variable_options
    #  update the variable options of the menu
    def update_variable_options(self, userdata):
        try:
            idx = userdata.sequence_idx
            fname = userdata.sequence_item
            if self.sequence[idx] == "Add" and fname != "Empty":
                self.sequence.append("Add")
                self.sequence[idx] = fname
            elif self.sequence[idx] != "Add" and fname == "Empty":
                del self.sequence[idx]
            elif self.sequence[idx] != "Add" and fname != "Empty":
                self.sequence[idx] = fname
        except KeyError:
            pass
        userdata.sequence = self.sequence[:-1]
        return self.sequence
