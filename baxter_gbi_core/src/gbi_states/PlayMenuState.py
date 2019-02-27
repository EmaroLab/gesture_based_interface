# -*- coding: latin-1 -*-
## @package PlayMenuState
## This package describes the structure of the play menu state 

from MenuState import MenuState
from baxter_gbi_pbr_srvs.srv import ListFiles
import rospy

##  PlayMenuState
#   inerithed form MenuState
class PlayMenuState(MenuState):
    ## constructor
    #  @param trigger_event istance of FsmEvent class
    def __init__(self, trigger_event):
        outcomes = ['back',
                    'remove']

        MenuState.__init__(self,
                           outcomes,
                           trigger_event,
                           'Playback menu',
                           fixed_options=['back','remove'])

        rospy.wait_for_service('files')
        self.list = rospy.ServiceProxy('files', ListFiles)

    ## method update_variable_options
    #  @param userdata 
    #
    #  override of MenuState.update_variable_options
    #  update the variable options of the menu
    def update_variable_options(self, userdata):
        # call a service to ask and recieve the data
        # or parameter server or message
        try:
            list = self.list()
            #print list
            return list.list_files
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return []

    def on_variable_selection(self, index, item, userdata):
        userdata.selection = item
        return 'selection'