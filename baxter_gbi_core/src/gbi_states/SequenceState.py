from ActionState import ActionState
import rospy
import actionlib
from baxter_gbi_pbr_msgs.msg import playbackAction, playbackGoal
from baxter_gbi_pbr_srvs.srv import *

class SequenceState(ActionState):
    def __init__(self, trigger_event):
        input_keys = ['sequence']

        ActionState.__init__(self,
                             outcomes=['pause'],
                             trigger_event=trigger_event,
                             status='play',
                             output_keys=[],
                             input_keys=input_keys)
        self.index=0
    
    def fun(self,sequence,vel):
        self.playback.cancel_goal()
        self.goal.msg.filename=sequence
        self.goal.msg.loops=1
        self.goal.msg.scale_vel=vel
        self.playback.send_goal(self.goal,self.cb_done,None,self.feedback_cb)

    def cb_done(self,status, result):
        if  self.index<=len(self.sequence):
            self.index+=1
            self.fun(self.sequence[self.index],100)
        else:
            self.signal('finished')

    def set_status(self):
        if self.progress >= 0:
            return self.goal.msg.filename + "%d%% completed" % self.progress
        else:
            return "reaching initial position... "
    
    def execute(self,userdata):
        self.sequence = userdata.sequence
        self.fun(self.sequence[self.index],100)
        return ActionState.execute(self,userdata)
    
    def action_1(self,userdata):
        self.cb_done(None,None)
        return None