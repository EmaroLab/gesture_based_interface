## @package MacroState
## This package describes the structure
#  of the macro state 

from ActionState import ActionState
import rospy
import actionlib
from baxter_gbi_pbr_msgs.msg import playbackAction, playbackGoal
from baxter_gbi_pbr_srvs.srv import *

class MacroState(ActionState):
    def __init__(self, trigger_event):
        input_keys = ['filename']

        ActionState.__init__(self,
                             outcomes=['pause'],
                             trigger_event=trigger_event,
                             status='pause',
                             output_keys=[],
                             input_keys=input_keys)
        
        

    def cb_done(self, status, result):
        return None

    def fun(self,filename,vel):
        if (filename!="Empty"):
            self.playback.cancel_goal()
            self.goal.msg.filename=filename
            self.goal.msg.loops=1
            self.goal.msg.scale_vel=vel
            self.playback.send_goal(self.goal,self.cb_done,None,self.feedback_cb)

    def set_status(self):
        if self.progress >= 0:
            return self.goal.msg.filename + "%d%% completed" % self.progress
        else:
            return "reaching initial position... "

    def action_4(self,userdata):
        self.fun(userdata.filename[0],100)
        return None

    def action_3(self,userdata):
        self.fun(userdata.filename[1],100)
        return None

    def action_2(self,userdata):
        self.fun(userdata.filename[2],100)
        return None

    def action_1(self,userdata):
        self.fun(userdata.filename[3],100)
        return None