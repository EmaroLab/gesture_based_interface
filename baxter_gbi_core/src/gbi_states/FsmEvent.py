# -*- coding: latin-1 -*-
## @package BlockingState
## This package describes the general blocking state 

import threading

## FsmEvent
# inherited form smach.State
class FsmEvent:
    ## constructor
    def __init__(self):
        self.trigger = threading.Event()
        self.event_id = None

    ## method wait
    #
    # method to wait for an event
    def wait(self):
        self.trigger.clear()
        self.trigger.wait()
        return self.event_id
    
    ## method signal
    #  sets the trigger
    #
    #  @param event_id event received
    def signal(self, event_id):
        self.event_id = event_id
        #rospy.loginfo("Received event " + self.event_id)
        self.trigger.set()
