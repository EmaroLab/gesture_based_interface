# -*- coding: latin-1 -*-
## @package ActionState
## This package describes the structure of the action state

from ActionState import ActionState

## SequenceState
# inherited form ActionState
class SequenceState(ActionState):
    ## constructor
    # @param trigger_event instance of the class FsmEvent
    def __init__(self, trigger_event):
        input_keys = ['sequence', 'new_sequence']
        output_keys = ['filename', 'new_sequence']

        ActionState.__init__(self,
                             outcomes=['play'],
                             trigger_event=trigger_event,
                             status='wait',
                             output_keys=output_keys,
                             input_keys=input_keys)
        self.index = -1
        self.sequence = []

    ## method play_file
    # @param userdata
    # @param index
    #
    # plays the file at the index passed
    # as parameter
    @staticmethod
    def play_file(userdata, index):
        userdata.filename = userdata.sequence[index]
        return "play"

    ## method new_sequence
    # @param userdata
    #
    # creates a new sequence
    def new_sequence(self, userdata):
        try:
            new = userdata.new_sequence
            userdata.new_sequence = False
            print "New sequence "
            print new
            return new
        except KeyError:
            return False

    ## method execute
    # @param userdata
    #
    # executes the sequence
    def execute(self, userdata):
        if self.new_sequence(userdata):
            self.index = -1
        if self.index < len(userdata.sequence)-1:
            self.index += 1
            return self.play_file(userdata, self.index)
        else:
            return "done"
