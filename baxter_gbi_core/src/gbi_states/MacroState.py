# -*- coding: latin-1 -*-
## @package MacroState
## This package describes the structure of the macro state

from ActionState import ActionState

class MacroState(ActionState):
    def __init__(self, trigger_event):
        input_keys = ['filenames']
        output_keys = ['filename']

        ActionState.__init__(self,
                             outcomes=['play'],
                             trigger_event=trigger_event,
                             status='wait',
                             output_keys=output_keys,
                             input_keys=input_keys,
                             fixed_options = ['1','2','3'])

    ## method set_status
    #
    # sets the status as waiting for
    # action request
    def set_status(self):
        return "waiting for action request"

    ## method play_file
    # plays the file corresponding to the one
    # at the index passed as parameter
    #
    # @param userdata
    # @param index
    @staticmethod
    def play_file(userdata, index):
        if userdata.filenames[index] != "Empty":
            userdata.filename = userdata.filenames[index]
            return "play"
        return None

    ## method action_1
    # override of BlockingState.action_1
    # where action_1 is assumed to be
    # "play the first macro"
    #
    # @param userdata
    def action_2(self, userdata):
        return self.play_file(userdata, 0)

    ## method action_
    # override of BlockingState.action_2
    # where action_2 is assumed to be
    # "play the second macro"
    #
    # @param userdata
    def action_3(self, userdata):
        return self.play_file(userdata, 1)

    ## method action_3
    # override of BlockingState.action_3
    # where action_3 is assumed to be
    # "play the third macro"
    #
    # @param userdata
    def action_4(self, userdata):
        return self.play_file(userdata, 2)

    ## method action_4
    # override of BlockingState.action_4
    # where action_4 is assumed to be
    # "play the fourth macro"
    #
    # @param userdata
    def action_1(self, userdata):
        return 'done'

    ## method user_left
    # called when the user leaves
    #
    # @param userdata
    def user_left(self, userdata):
        return 'user_missed'
