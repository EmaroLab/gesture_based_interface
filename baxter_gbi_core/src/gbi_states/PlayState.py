# -*- coding: latin-1 -*-
## @package PlayState
## This package describes the general structure of the play state

from PlaybackState import PlaybackState
import rospy

## PlayState
# inherited from PlaybackState
class PlayState(PlaybackState):
    def __init__(self, trigger_event):
        input_keys = ['filename', 'resume']
        output_keys = ['resume']

        PlaybackState.__init__(self,
                               outcomes=["pause"],
                               trigger_event=trigger_event,
                               status='play',
                               output_keys=output_keys,
                               input_keys=input_keys,
                               fixed_options=['pause'])
        self.progress = 0

    ## method cb_done
    # called when the playback is finished
    #
    # @param status
    # @param result
    def cb_done(self, status, result):
        PlaybackState.end = True
        if not PlaybackState.killing:
            self.signal("finished")

    ## method cb_feedback
    # gives info about the state of the play
    #
    # @param result
    def cb_feedback(self, result):
        self.progress = result.percent_complete
        if self.is_running():
            self.publish_state()

    ## method pause
    #
    # pauses the reproduction of the action
    def pause(self):
        try:
            self.pause_resume(1)
            return 'pause'
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return None

    ## method action_6
    # override of BlockingState.action_6
    # where action_6 is assumed to be "pause"
    #
    # @param userdata
    def action_2(self, userdata):
        return self.pause()

    ## method set_status
    #
    # returns the percentage of completion of
    # the playback
    def set_status(self):
        if self.progress >= 0:
            return "%d%% completed" % self.progress
        else:
            return "reaching initial position... "

    ## method new_instance
    # new instance of play
    #
    # @param userdata
    def new_instance(self, userdata):
        try:
            new = not userdata.resume
            userdata.resume = False
            print "New play: ",
            print new
            return new
        except KeyError:
            return True

    ## method play
    # plays the corresponding file
    #
    # @param filename
    def play(self, filename):
        PlaybackState.end = False
        PlaybackState.killing = False
        self.progress = 110
        self.goal.msg.filename = filename
        self.goal.msg.loops = 1
        self.goal.msg.scale_vel = 100
        self.playback.send_goal(self.goal, self.cb_done, None, self.cb_feedback)

    ## method execute
    # executes the action
    #
    # @param userdata
    def execute(self, userdata):
        if self.new_instance(userdata):
            self.play(userdata.filename)
        return PlaybackState.execute(self, userdata)

