# -*- coding: latin-1 -*-
## @package RecordState
## This package describes the structure of the record state
from ActionState import ActionState

import rospy
from baxter_gbi_pbr_srvs.srv import RecordStart, RecordStop, Gripper
from baxter_core_msgs.msg import DigitalIOState

import os
debug = os.environ.get('BGI_DEBUG')

## RecordState
# inherited from ActionState
class RecordState(ActionState):
    ## constructor
    #  @param trigger_event instance of FsmEvent class
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
        self.go_left = 1
        self.go_right = 1

        rospy.Subscriber("/robot/digital_io/left_lower_cuff/state", DigitalIOState, self.callback_button_left, callback_args=[self])
        rospy.Subscriber("/robot/digital_io/right_lower_cuff/state", DigitalIOState, self.callback_button_right, callback_args=[self])

    ## method user_left
    # called when the user leaves
    #
    # @param userdata
    def user_left(self, userdata):
        return None

    ## method action_1
    # override of BlockingState.action_1
    # where action_1 is assumed to be
    # "close right gripper"
    #
    # @param userdata
    def action_3(self, userdata):
        self.right_grip = not self.right_grip
        self.gripper("right", 100 if self.right_grip else 0)
        return None

    ## method action_2
    # override of BlockingState.action_2
    # where action_2 is assumed to be
    # "close left gripper"
    #
    # @param userdata
    def action_4(self, userdata):
        self.left_grip = not self.left_grip
        self.gripper("left", 100 if self.left_grip else 0)
        return None

    def action_1(self, userdata):
        if self.go_left == 0 and self.go_right == 0:
            self.record_stop()
            return 'done'


    def callback_button_left(self, data, params):
        self.go_left = data.state


    def callback_button_right(self, data, params):
        self.go_right = data.state

    ## method execute
    # starts recording
    #
    # @param userdata
    def execute(self, userdata):
        self.record_start(userdata.filename, 100)
        return ActionState.execute(self, userdata)
