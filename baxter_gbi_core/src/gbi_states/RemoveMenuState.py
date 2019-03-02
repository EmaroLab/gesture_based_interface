# -*- coding: latin-1 -*-
## @package RemoveMenuState
## This package describes the structure of the play menu state 

from FileMenuState import FileMenuState
from baxter_gbi_pbr_srvs.srv import DeleteFile
import rospy

import os
debug = os.environ.get('BGI_DEBUG')

## RemoveMenuState
#  inherited form FileMenuState
class RemoveMenuState(FileMenuState):
    #  constructor
    #  @param trigger_event instance of FsmEvent class
    def __init__(self, trigger_event):
        outcomes = ['back']
        FileMenuState.__init__(self,
                               outcomes,
                               trigger_event,
                               'Select the recording to delete',
                               input_keys=[],
                               fixed_options=['back'])
        if not debug:
            rospy.wait_for_service('delete_file')
        self.delete = rospy.ServiceProxy('delete_file', DeleteFile)

    #  method update_variable_options
    #  @param index
    #  @param item
    #  @param userdata 
    #  
    #  override of MenuState.on_variable_selection
    def on_variable_selection(self, index, item, userdata):
        self.delete(item)
        del self.variable_options[index]
        self.selection = 0
        return None
