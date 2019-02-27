# -*- coding: latin-1 -*-
## @package SubMacroMenuState
## This package describes the structure of the sequence menu state 

from MenuState import MenuState
from baxter_gbi_pbr_srvs.srv import ListFiles
import rospy

##  SubMacroMenuState
#   inerithed form MenuState
class SubMacroMenuState(MenuState):
    ## constructor
    #  @param trigger_event istance of FsmEvent class
    def __init__(self, trigger_event):
        outcomes = ['back']
        MenuState.__init__(self,
                           outcomes,
                           trigger_event,
                           'Macro menu - selection',
                           input_keys=['macro_idx'],
                           output_keys=['macro_idx_out'],
                           fixed_options=['back', 'clean'])
        self.list = rospy.ServiceProxy('files', ListFiles)

    ## method update_variable_options
    #  @param userdata 
    #
    #  override of MenuState.update_variable_options
    #  update the variable options of the menu
    def update_variable_options(self, userdata):
        userdata.macro_idx_out = userdata.macro_idx
        list = self.list()
        return list.list_files

    def on_variable_selection(self, index, item, userdata):
        userdata.selection = item
        return 'selection'

        ## method on_fixed_selection
    #  @param userdata 
    #  @param index
    #  @param item
    #  
    #  override of MenuState.on_fixed_selection
    def on_fixed_selection(self, index, item, userdata):
        if item == 'clean':
            userdata.selection = ''
            return 'selection'
        else:
            return item
