from MenuState import MenuState

class MainMenuState(MenuState):
    def __init__(self, trigger_event):
        outcomes = ['user_missed',
                    'play_selected',
                    'record_selected',
                    'macro_selected',
                    'sequence_selected',
                    'preempted']

        MenuState.__init__(self,
                           outcomes,
                           trigger_event,
                           'Main menu',
                           fixed_options=[])

    def update_variable_options(self, userdata):
        return ['play',
                'record',
                'macro',
                'sequence']

    def on_variable_selection(self, index, item, userdata):
        return item + '_selected'

    def on_fixed_selection(self, index, item, userdata):
        raise RuntimeError  # should never be called in main menu