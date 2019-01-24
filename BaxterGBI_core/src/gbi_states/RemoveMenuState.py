from MenuState import MenuState

class RemoveMenuState(MenuState):
    def __init__(self, trigger_event):
        outcomes = ['back']

        MenuState.__init__(self,
                           outcomes,
                           trigger_event,
                           'Select the recording to delete',
                           input_keys=[],
                           fixed_options=['back'])

    def update_variable_options(self, userdata):
        return ['demo record']  # TODO: ask PBR the list of files

    def on_variable_selection(self, index, item, userdata):
        # TODO: ask PBR to delete the record
        return 'back'