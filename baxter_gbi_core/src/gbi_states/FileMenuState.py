# -*- coding: latin-1 -*-
## @package PlayMenuState
## This package describes the structure of the play menu state 

from MenuState import MenuState
from baxter_gbi_pbr_srvs.srv import ListFiles
import rospy

##  PlayMenuState
#   inerithed form MenuState
class FileMenuState(MenuState):
    ## constructor
    #  @param trigger_event istance of FsmEvent class
    def __init__(self, outcomes, trigger_event, page_title, output_keys=[], input_keys=[], fixed_options=[]):
        MenuState.__init__(self,
                           outcomes=outcomes,
                           trigger_event=trigger_event,
                           page_title=page_title,
                           output_keys=output_keys,
                           input_keys=input_keys,
                           fixed_options=fixed_options)

        rospy.wait_for_service('files')
        self.list = rospy.ServiceProxy('files', ListFiles)

    ## method update_variable_options
    #  @param userdata 
    #
    #  override of MenuState.update_variable_options
    #  update the variable options of the menu
    def update_variable_options(self, userdata):
        # call a service to ask and receive the data
        # or parameter server or message
        list = self.list()
        return list.list_files