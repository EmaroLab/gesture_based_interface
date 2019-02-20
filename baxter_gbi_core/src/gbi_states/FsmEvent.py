## @package BlockingState
## This package describes the general blocking state 

import rospy
import threading

##  FsmEvent
#   inerithed form smach.State
class FsmEvent:
    ## constructor
    def __init__(self):
        self.trigger = threading.Event()
        self.event_id = None
        

    ## method wait
    #  method to wait an event
    def wait(self):
        self.trigger.clear()
        self.trigger.wait()
        return self.event_id
    
    ## method signal
    #  @param event_id event received
    #
    #  set the trigger 
    def signal(self, event_id):
        self.event_id = event_id
        rospy.loginfo("Received event " + self.event_id)
        self.trigger.set()
