# -*- coding: latin-1 -*-
from PlaybackState import PlaybackState


class PauseState(PlaybackState):
    def __init__(self, trigger_event):
        PlaybackState.__init__(self,
                             outcomes=['resume'],
                             trigger_event=trigger_event,
                             status='pause',
                             output_keys=['resume'],
                             input_keys=[])

    def resume(self):
        try:
            self.pause_resume(0)
            return 'resume'
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
            return None

    def action_6(self, userdata):
        userdata.resume = True
        return self.resume()

    def set_status(self):
        return "paused"

