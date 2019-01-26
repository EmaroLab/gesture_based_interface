## @package MacroState
## This package describes the structure
#  of the macro state 

from ActionState import ActionState

class MacroState(ActionState):
    def __init__(self, trigger_event):
        input_keys = ['filename']

        ActionState.__init__(self,
                             outcomes=[],
                             trigger_event=trigger_event,
                             action='Record mode',
                             output_keys=[],
                             input_keys=input_keys)