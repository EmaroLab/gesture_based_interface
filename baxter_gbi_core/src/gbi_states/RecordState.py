from ActionState import ActionState
import rospy
import actionlib
from baxter_gbi_pbr_msgs.msg import *
from baxter_gbi_pbr_srvs.srv import RecordStart, RecordStop

class RecordState(ActionState):
    def __init__(self, trigger_event):
        input_keys = ['filename']

        ActionState.__init__(self,
                             outcomes=[],
                             trigger_event=trigger_event,
                             status='record',
                             output_keys=[],
                             input_keys=input_keys)
        self.record_start = rospy.ServiceProxy('record_start', RecordStart)
        self.record_stop=rospy.ServiceProxy('record_stop', RecordStop)
        self.progress = 0

    def user_left(self, userdata):
        return None
    
    def action_6(self,userdata):
        self.record_stop()
        return 'done'


    def execute(self,userdata):
        self.record_start('record_1','100')
        return ActionState.execute(userdata)