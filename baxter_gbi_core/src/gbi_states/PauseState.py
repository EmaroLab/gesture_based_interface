# -*- coding: latin-1 -*-
## @package PauseState
## This package describes the general structure of the pause state

from PlaybackState import PlaybackState

## PauseState
# inherited form PlaybackState
class PauseState(PlaybackState):
    ## constructor
    # @param trigger event event which triggers the state
    def __init__(self, trigger_event):
        PlaybackState.__init__(self,
                             outcomes=['resume'],
                             trigger_event=trigger_event,
                             status='pause',
                             output_keys=['resume'],
                             input_keys=[],
                             fixed_options = ['resume'])

    ## method resume
    #
    # resumes the playback of the action
    def resume(self):
        try:
            self.pause_resume(0)
            return 'resume'
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
            return None

    ## method action_6
    # override of BlockingState.action_6
    # where action_6 is assumed to be "resume"
    #
    # @param userdata
    def action_2(self, userdata):
        userdata.resume = True
        return self.resume()

    ## method set_status
    #
    # sets the status
    def set_status(self):
        return "paused"

