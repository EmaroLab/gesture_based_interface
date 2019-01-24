from MenuState import MenuState


class SubSequenceMenuState(MenuState):
    def __init__(self, trigger_event):
        outcomes = ['back']

        MenuState.__init__(self,
                           outcomes,
                           trigger_event,
                           'Sequence menu - selection',
                           input_keys=['sequence_idx'],
                           fixed_options=['back', 'clean'])

    def update_variable_options(self, userdata):
        return ['demo record']  # TODO: ask PBR the list of files

    def on_fixed_selection(self, index, item, userdata):
        if item == 'clean':
            userdata.selection = ''
            return 'selection'
        else:
            return item