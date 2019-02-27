# -*- coding: latin-1 -*-
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
        input_keys = ['filenames']
        output_keys = ['filename']

        ActionState.__init__(self,
                             outcomes=['play'],
                             trigger_event=trigger_event,
                             status='wait',
                             output_keys=output_keys,
                             input_keys=input_keys)

    def set_status(self):
        return "waiting for action request"

    def action_1(self, userdata):
        if userdata.filenames[0] != "Empty":
            userdata.filename = userdata.filenames[0]
            return "play"
        return None

    def action_2(self, userdata):
        if userdata.filenames[1] != "Empty":
            userdata.filename = userdata.filenames[1]
            return "play"
        return None

    def action_3(self, userdata):
        if userdata.filenames[2] != "Empty":
            userdata.filename = userdata.filenames[2]
            return "play"
        return None

    def action_4(self, userdata):
        if userdata.filenames[3] != "Empty":
            userdata.filename = userdata.filenames[3]
            return "play"
        return None

    def action_5(self, userdata):
        return 'done'

    def user_left(self, userdata):
        return 'user_missed'
