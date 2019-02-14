## @package RecordState
## This package describes the structure of the record state 

from ActionState import ActionState

##  RecordMenuState
#   inerithed form ActionState
class RecordState(ActionState):
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
