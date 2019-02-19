from ActionState import ActionState
from baxter_gbi_pbr_srvs.srv import Playback
import baxter_gbi_pbr_msgs.msg as playback_msg
import actionlib
from baxter_gbi_pbr_msgs.msg import PlaybackAction PlaybackGoal

class PlayState(ActionState):
    def __init__(self, trigger_event):
        input_keys = ['filename']

        ActionState.__init__(self,
                             outcomes=['pause'],
                             trigger_event=trigger_event,
                             action='Playback mode',
                             output_keys=[],
                             input_keys=input_keys)

        rospy.wait_for_service('Playback')
        self.playback = actionlib.SimpleActionClient('playback', Playback)
        self.msg=playback_msg.playback_msg
        self.goal=PlaybackGoal()


    def action_6(self,userdata):
        
        self.playback.wait_for_service()
        self.goal.filename=userdata.filename
        self.goal.loops=1;
        self.goal.scale_vel=100;
        self.send_goal(self.goal)v
        return 'pause'


    def execute(self,userdata)
        self.playback.wait_for_service()
        self.goal.filename=userdata.filename
        self.goal.loops=1;
        self.goal.scale_vel=100;
        self.send_goal(self.goal)
        ActionState.execute(userdata)