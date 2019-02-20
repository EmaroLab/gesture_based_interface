from ActionState import ActionState
import rospy
import actionlib
from baxter_gbi_pbr_msgs.msg import *

class RecordState(ActionState):
    def __init__(self, trigger_event):
        input_keys = ['filename']

        ActionState.__init__(self,
                             outcomes=[],
                             trigger_event=trigger_event,
                             status='record',
                             output_keys=[],
                             input_keys=input_keys)
        self.goal = recordGoal()
        self.progress = 0

    def user_left(self, userdata):
        return None

    def feedback_cb(self,result):
        self.progress = result

    def done_cb(self,userdata):
        self.signal('finished')

    def execute(self,userdata):
        self.rec = actionlib.SimpleActionClient('playback', recordAction)
        self.rec.wait_for_service()
        self.goal.filename = userdata.filename
        self.goal.record_rate=100
        self.rec.send_goal(self.goal, self.done_cb, None, self.feedback_cb)
        return ActionState.execute(userdata)