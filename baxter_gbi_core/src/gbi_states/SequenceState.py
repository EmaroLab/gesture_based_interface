## @package SequenceState
## This package describes the structure of the sequence state 

from ActionState import ActionState

##  SequenceState
#   inerithed form ActionState
class SequenceState(ActionState):
    ## constructor
    #  @param trigger_event istance of FsmEvent class
    def __init__(self, trigger_event):
        input_keys = ['filename']
        ActionState.__init__(self,
                             outcomes=[],
                             trigger_event=trigger_event,
                             action='Record mode',
                             output_keys=[],
                             input_keys=input_keys)
