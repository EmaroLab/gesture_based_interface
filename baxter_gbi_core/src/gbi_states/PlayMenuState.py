## @package PlayMenuState
## This package describes the structure
#  of the play menu state 

from MenuState import MenuState

##  PlayMenuState
#   inerithed form MenuState
class PlayMenuState(MenuState):
    ## the constructor
    #  @param trigger_event istance of the class FsmEvent
    def __init__(self, trigger_event):
        outcomes = ['back',
                    'remove']

        MenuState.__init__(self,
                           outcomes,
                           trigger_event,
                           'Playback menu',
                           fixed_options=['back'])

    ## method update_variable_options
    #  @param userdata 
    #
    #  override of MenuState.update_variable_options
    #  update the variable options of the menu
    def update_variable_options(self, userdata):
        if self.preempt_requested():
            return 'preempted'
        event_id = self._trigger_event.wait()
        if self.preempt_requested():
            return 'preempted'
        # call a service to ask and recieve the data
        # or parameter server or message

        return ['demo record']  # TODO: ask PBR the list of files