from ActionState import ActionState
import rospy
import actionlib
from baxter_gbi_pbr_msgs.msg import playbackAction, playbackGoal
from baxter_gbi_pbr_srvs.srv import *

class PlayState(ActionState):
    def __init__(self, trigger_event):
        input_keys = ['filename','resume']

        ActionState.__init__(self,
                             outcomes=['pause'],
                             trigger_event=trigger_event,
                             status='play',
                             output_keys=[],
                             input_keys=input_keys)
        self.goal = playbackGoal()
        rospy.wait_for_service('pause_resume')
        self.pause = rospy.ServiceProxy('pause_resume',PauseResume)
        self.playback = actionlib.SimpleActionClient('playback', playbackAction)
        self.playback.wait_for_server()
        self.progress = 0

    def action_5(self,userdata):
        self.playback.cancel_goal()
        while(self.progress<100):
            print self.progress
        self.signal('finished')

    def action_6(self,userdata):
        try:
            pause=self.pause(1)
            return 'pause'
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return None
    
    def user_left(self, userdata):
        self.playback.cancel_goal()
        while(self.progress<100):
            print self.progress
        return 'user_missed'
    
    def cb_done(self,status,result):
        self.signal('finished')
        
    def feedback_cb(self,result):
        if(self.is_running()):
            self.progress = result.percent_complete
            self.publish_state()

    def set_status(self):
        if self.progress>=0:
            return "%d%% completed" % self.progress
        else:
            return "reaching initial position... "
            
            
    def execute(self,userdata):
        new_instance = True
        try:
            if (userdata.resume):
                new_instance = False
        except KeyError:
            pass
        if(new_instance):
            self.progress = 110
            self.goal.msg.filename = userdata.filename
            self.goal.msg.loops = 1;
            self.goal.msg.scale_vel = 100;
            self.playback.send_goal(self.goal, self.cb_done, None, self.feedback_cb)
        return ActionState.execute(self, userdata)