#!/usr/bin/env python
from ActionState import ActionState
import rospy
from baxter_gbi_pbr_srvs.srv import*

class PlayPause(ActionState):
    def __init__(self, trigger_event):
        ActionState.__init__(self,
                             outcomes=['resume'],
                             trigger_event=trigger_event,
                             status='pause',
                             output_keys=[],
                             input_keys=[])
        self.resume=rospy.ServiceProxy('pause_resume',PauseResume)

    def action_6(self,userdata):
        try:
            self.resume(0)
            return 'resume'
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return None

    def set_status(self):
        return "paused"