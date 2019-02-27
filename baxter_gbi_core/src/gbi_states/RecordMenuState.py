# -*- coding: latin-1 -*-
## @package RecordMenuState
## This package describes the structure of the record menu state 

from MenuState import MenuState
import rospy
from baxter_gbi_pbr_srvs.srv import ListFiles

##  RecordMenuState
#   inerithed form MenuState
class RecordMenuState(MenuState):
    ## constructor
    #  @param trigger_event istance of FsmEvent class
    def __init__(self, trigger_event):
        outcomes = ['back',
                    'remove']

        MenuState.__init__(self,
                           outcomes,
                           trigger_event,
                           'Recording menu',
                           fixed_options=['back', 'remove', 'new'])

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
            # print list
            return list.list_files
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
            return []


    ## method on_fixed_selection
    #  @param userdata 
    #  @param index
    #  @param item
    #  
    #  override of MenuState.on_fixed_selection
    def on_fixed_selection(self, index, item, userdata):
        if item == 'new':
            userdata.selection = "new_name"  # TODO: generate new unique name for recording
            return 'selection'
        else:
            return MenuState.on_fixed_selection(index, item, userdata)