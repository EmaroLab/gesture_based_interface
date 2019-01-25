from MenuState import MenuState

class PlayMenuState(MenuState):
    def __init__(self, trigger_event):
        outcomes = ['back',
                    'remove']

        MenuState.__init__(self,
                           outcomes,
                           trigger_event,
                           'Playback menu',
                           fixed_options=['back'])

    def update_variable_options(self, userdata):
        return ['demo record']  # TODO: ask PBR the list of files