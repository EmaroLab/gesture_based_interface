from ActionState import ActionState

class MacroPause(ActionState):
    def __init__(self, trigger_event):
        ActionState.__init__(self,
                             outcomes=['resume'],
                             trigger_event=trigger_event,
                             status='PausePlay mode',
                             output_keys=[],
                             input_keys=[])

    def action_6(self,userdata):
        return 'resume'