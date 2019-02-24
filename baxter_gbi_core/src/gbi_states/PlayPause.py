#!/usr/bin/env python
from ActionState import ActionState
import rospy
import actionlib
from baxter_gbi_pbr_msgs.msg import playbackAction, playbackGoal
from baxter_gbi_pbr_srvs.srv import*

class PlayPause(ActionState):
    def __init__(self, trigger_event):
        ActionState.__init__(self,
                             outcomes=['resume','rec_s'],
                             trigger_event=trigger_event,
                             status='pause',
                             output_keys=['resume'],
                             input_keys=[])
        

    def action_6(self,userdata):
        try:
            self.pause(0)
            userdata.resume=True
            return 'resume'
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return None

    def action_4(self,userdata):
        return 'rec_s'
    
    def set_status(self):
        return "paused"