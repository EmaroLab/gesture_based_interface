from MenuState import MenuState

class RecordMenuState(MenuState):
    def __init__(self, trigger_event):
        outcomes = ['back',
                    'remove']

        MenuState.__init__(self,
                           outcomes,
                           trigger_event,
                           'Recording menu',
                           fixed_options=['back', 'remove', 'new'])

    def update_variable_options(self, userdata):
        return ['demo record']  # TODO: ask PBR the list of files

    def on_fixed_selection(self, index, item, userdata):
        if item == 'new':
            userdata.selection = "new_name"  # TODO: generate new unique name for recording
            return 'selection'
        else:
            return item