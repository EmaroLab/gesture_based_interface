from ActionState import ActionState
import rospy
import actionlib
from baxter_gbi_pbr_msgs.msg import playbackAction, playbackGoal
from baxter_gbi_pbr_srvs.srv import *

class SequenceState(ActionState):
    def __init__(self, trigger_event):
        input_keys = ['filename','m_index']

        ActionState.__init__(self,
                             outcomes=['pause'],
                             trigger_event=trigger_event,
                             status='play',
                             output_keys=[],
                             input_keys=input_keys)
        self.goal = playbackGoal()
        self.pause = rospy.ServiceProxy('pause_resume', PauseResume)
        self.progress = 0
        self.index=0

    def action_6(self,userdata):
        try:
            pause=self.pause(1)
            return 'pause'
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return None
    
    def fun(self,filename,vel):
        self.playback = actionlib.SimpleActionClient('playback', playbackAction)
        self.playback.wait_for_service()
        self.goal.filename=filename
        self.goal.loops=1;
        self.goal.scale_vel=vel;
        self.playback.send_goal(self.goal,self.done_cb,None,self.feedback_cb)

    def cb_done(self, userdata):
        if self.index<=userdata.m_index:
            self.index+=1
            self.fun(userdata.filename[self.index],100)
        else:
            self.signal('finished')

    def feedback_cb(self, result):
        self.progress = result
        self.publish_state()

    def set_status(self):
        return "%d% completed" % self.progress
    
    def execute(self,userdata):
        self.fun(userdata.filename[self.index],100)
        return ActionState.execute(userdata)