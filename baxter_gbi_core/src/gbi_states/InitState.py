## @package InitState
#  The package defines the structure of the IINIT state

 
import rospy
import smach

##  InitState
#   outcomes: (config_available,config_missing,
#  'preempted')
class InitState(smach.State):
    ## The constructor
    def __init__(self):
        
        outcomes = list(['config_available',
                         'config_missing',
                         'preempted'])
        smach.State.__init__(
            self,
            outcomes)

    ##  method execute
    #   @param self the object pointer
    #   @param userdata the data come from the prevoius state
    def execute(self, userdata):
        
        if self.preempt_requested():
            return 'preempted'
        for i in range(1, 7):
            key = "key_" + str(i) + "_topics"
            if not rospy.has_param(key) or not rospy.get_param(key):
                return 'config_missing'
        return 'config_available'