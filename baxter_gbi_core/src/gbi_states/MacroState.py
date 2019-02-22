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
        self.goal=playbackGoal()
        self.goal=self.goal.msg
        self.pause=rospy.ServiceProxy('pause_resume',PauseResume)
        self.playback = actionlib.SimpleActionClient('playback', playbackAction)
        self.playback.wait_for_server()
        self.progress = 0
        self.current_file = ""

    def action_6(self,userdata):
        try:
            pause=self.pause(1)
            return 'pause'
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return None

    def cb_done(self, status, result):
        self.signal('finished')

    def feedback_cb(self, result):
        self.progress = result.percent_complete
        self.publish_state()

    def fun(self,filename,vel):
        self.goal.filename=filename
        self.goal.loops=1;
        self.goal.scale_vel=vel;
        self.playback.send_goal(self.goal,self.cb_done,None,self.feedback_cb)

    def action_5(self,userdata):
        return 'done'

    def action_4(self,userdata):
        self.fun(userdata.filename[2],100)
        return None

    def action_3(self,userdata):
        self.fun(userdata.filename[3],100)
        return None

    def action_2(self,userdata):
        self.fun(userdata.filename[4],100)
        return None

    def action_1(self,userdata):
        self.fun(userdata.filename[5],100)
        return None