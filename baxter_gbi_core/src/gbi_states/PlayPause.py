#!/usr/bin/env python
from ActionState import ActionState

class PlayPause(ActionState):
    def __init__(self, trigger_event):
        ActionState.__init__(self,
                             outcomes=['resume'],
                             trigger_event=trigger_event,
                             action='PausePlay mode',
                             output_keys=[],
                             input_keys=[])

    def action_6(self,userdata):
        return 'resume'