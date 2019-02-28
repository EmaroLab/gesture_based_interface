# -*- coding: latin-1 -*-
from ActionState import ActionState
import rospy
from baxter_gbi_pbr_srvs.srv import RecordStart, RecordStop, Gripper

class RecordState(ActionState):
    def __init__(self, trigger_event):

        ActionState.__init__(self,
                             outcomes=[],
                             trigger_event=trigger_event,
                             status='record',
                             output_keys=[],
                             )
        rospy.wait_for_service('record_start')
        self.record_start = rospy.ServiceProxy('record_start', RecordStart)
        rospy.wait_for_service('record_stop')
        self.record_stop=rospy.ServiceProxy('record_stop', RecordStop)
        rospy.wait_for_service('gripper')
        self.gripper = rospy.ServiceProxy('gripper', Gripper)

    def user_left(self, userdata):
        return None
    
    def action_6(self,userdata):
        self.record_stop()
        return 'done'

    def action_5(self,userdata):
        return None
    
    def action_1(self, userdata):
        self.gripper("right",100)
        return None

    def action_2(self, userdata):
        self.gripper("left",100)
        return None
    
    def execute(self,userdata):
        self.record_start('100') # il messaggio va cambiato perchè il nome lo decide il nodo pbr
        return ActionState.execute(self,userdata)