# -*- coding: latin-1 -*-
from ActionState import ActionState
import rospy
from baxter_gbi_pbr_srvs.srv import*

class Macro_SequencePause(ActionState):
    def __init__(self, trigger_event):
        ActionState.__init__(self,
                             outcomes=['resume'],
                             trigger_event=trigger_event,
                             status='PausePlay mode',
                             output_keys=[],
                             input_keys=[])
        
        
    def action_6(self, userdata):
        try:
            self.pasue(0)
            return 'resume'
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
            return None
        
    def set_status(self):
        return "paused"
