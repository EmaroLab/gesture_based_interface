from ActionState import ActionState

class SequenceState(ActionState):
    def __init__(self, trigger_event):
        input_keys = ['filename']

        ActionState.__init__(self,
                             outcomes=[],
                             trigger_event=trigger_event,
                             action='Record mode',
                             output_keys=[],
                             input_keys=input_keys)