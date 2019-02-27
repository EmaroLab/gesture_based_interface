# -*- coding: latin-1 -*-
## @package RemoveMenuState
## This package describes the structure of the play menu state 

from MenuState import MenuState
from baxter_gbi_pbr_srvs.srv import ListFiles, DeleteFile
import rospy
##  RemoveMenuState
#   inherited form MenuState
class RemoveMenuState(MenuState):
    ## constructor
    #  @param trigger_event istance of FsmEvent class
    def __init__(self, trigger_event):
        outcomes = ['back']

        MenuState.__init__(self,
                           outcomes,
                           trigger_event,
                           'Select the recording to delete',
                           input_keys=[],
                           fixed_options=['back'])
        rospy.wait_for_service('files')
        self.list = rospy.ServiceProxy('files', ListFiles)
        rospy.wait_for_service('delete_file')
        self.delete = rospy.ServiceProxy('delete_file', DeleteFile)
        

    ## method update_variable_options
    #  @param userdata 
    #
    #  override of MenuState.update_variable_options
    #  update the variable options of the menu
    def update_variable_options(self, userdata):
        try:
            list = self.list()
            #print list
            return list.list_files
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return []

    ## method update_variable_options
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
        
    
