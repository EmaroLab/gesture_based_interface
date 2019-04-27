# -*- coding: latin-1 -*-
## @package PlayMenuState
## This package describes the structure of the play menu state 

from MenuState import MenuState
from baxter_gbi_pbr_srvs.srv import ListFiles
import rospy

import os
debug = os.environ.get('BGI_DEBUG')

##  PlayMenuState
#   inherited form MenuState
class FileMenuState(MenuState):
    ## constructor
    # @param outcomes possible outcomes of the state
    # @param trigger_event object of the class FsmEvent
    # @param page_title title of the page
    # @param output_keys set of the data in output
    # @param input_keys set of the data in input
    # @param fixed_options fixed options of the menu
    def __init__(self, outcomes, trigger_event, page_title, output_keys=[], input_keys=[], fixed_options=[]):
        MenuState.__init__(self,
                           outcomes=outcomes,
                           trigger_event=trigger_event,
                           page_title=page_title,
                           output_keys=output_keys,
                           input_keys=input_keys,
                           fixed_options=fixed_options)

        if not debug:
            rospy.wait_for_service('files')
        self.list = rospy.ServiceProxy('files', ListFiles)

    ## method update_variable_options
    # override of MenuState.update_variable_options
    # update the variable options of the menu
    #
    # @param userdata
    def update_variable_options(self, userdata):
        # call a service to ask and receive the data
        # or parameter server or message
        reply = self.list()
        return reply.list_files
