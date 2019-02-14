## @package ActionState
## This package describes the structure of the action state 

from ActionState import ActionState

##  PlayMenuState
#   inherited form ActionState
class PlayState(ActionState):
    ## constructor
    #  @param trigger_event istance of FsmEvent class
    def __init__(self, trigger_event):
        input_keys = ['filename']

        ActionState.__init__(self,
                             outcomes=[],
                             trigger_event=trigger_event,
                             action='Playback mode',
                             output_keys=[],
                             input_keys=input_keys)
