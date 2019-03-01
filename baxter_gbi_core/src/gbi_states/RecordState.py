# -*- coding: latin-1 -*-
from ActionState import ActionState
import rospy
from baxter_gbi_pbr_srvs.srv import RecordStart, RecordStop, Gripper

import os
debug = os.environ.get('BGI_DEBUG')

class RecordState(ActionState):
    def __init__(self, trigger_event):
        input_keys = ['filename']
        ActionState.__init__(self,
                             outcomes=[],
                             trigger_event=trigger_event,
                             status='rec',
                             input_keys=input_keys,
                             output_keys=[])
        if not debug:
            rospy.wait_for_service('record_start')
        self.record_start = rospy.ServiceProxy('record_start', RecordStart)
        if not debug:
            rospy.wait_for_service('record_stop')
        self.record_stop=rospy.ServiceProxy('record_stop', RecordStop)
        if not debug:
            rospy.wait_for_service('gripper')
        self.gripper = rospy.ServiceProxy('gripper', Gripper)
        self.left_grip = True
        self.right_grip = True

    def user_left(self, userdata):
        return None
    
    def action_6(self,userdata):
        self.record_stop()
        return 'done'

    def action_1(self, userdata):
        self.right_grip = not self.right_grip
        self.gripper("right", 100 if self.right_grip else 0)
        return None

    def action_2(self, userdata):
        self.left_grip = not self.left_grip
        self.gripper("left", 100 if self.left_grip else 0)
        return None
    
    def execute(self, userdata):
        self.record_start(userdata.filename, 100)
        return ActionState.execute(self, userdata)
