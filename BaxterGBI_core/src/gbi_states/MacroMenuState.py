from MenuState import MenuState

class MacroMenuState(MenuState):
    def __init__(self, trigger_event):
        outcomes = ['play',
                    'back']

        MenuState.__init__(self,
                           outcomes,
                           trigger_event,
                           'Macro menu',
                           input_keys=['macro_idx', 'macro_filename'])

        self.macro_slots = [None, None, None, None, None]

    def update_variable_options(self, userdata):
        if userdata.macro_idx:  # TODO: check if it works
            if userdata.macro_filename:
                self.macro_slots[userdata.macro_idx] = userdata.macro_filename
            else:
                self.macro_slots[userdata.macro_idx] = None
        return self.macro_slots

    def on_variable_selection(self, index, item, userdata):
        userdata.selection = index
        return 'selected'

    # FIXME: 'play' outcome does not outputs macro configuration