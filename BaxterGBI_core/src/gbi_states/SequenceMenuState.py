from MenuState import MenuState

class SequenceMenuState(MenuState):
    def __init__(self, trigger_event):
        outcomes=['play',
                  'back']

        MenuState.__init__(self,
                           outcomes,
                           trigger_event,
                           'Sequence menu',
                           input_keys=['sequence_idx', 'sequence_filename'])

        self.sequence = [None]

    def update_variable_options(self, userdata):
        if userdata.sequence_idx:  # TODO: check if it works
            if userdata.sequence_filename:
                if not self.sequence[userdata.sequence_idx]:
                    self.sequence += None
                self.sequence[userdata.sequence_idx] = userdata.sequence_filename
            else:
                if self.sequence[userdata.sequence_idx]:
                    del self.sequence[userdata.sequence_idx]

        return self.sequence

    def on_variable_selection(self, index, item, userdata):
        userdata.selection = index
        return 'selected'

    # FIXME: 'play' outcome does not outputs sequence configuration