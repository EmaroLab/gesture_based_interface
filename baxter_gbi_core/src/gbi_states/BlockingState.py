## @package BlockingState
## This package describes the general blocking 
#  state 

import rospy
import smach
import baxter_gbi_core_msgs.msg as pub_status
import time

##  BlockingState
#   inerithed form smach.State
class BlockingState(smach.State):
    ## the constructor
    def __init__(self, outcomes, trigger_event, output_keys=[], input_keys=[]):
        outcomes = outcomes + ['preempted']
        smach.State.__init__(self,
                             outcomes,
                             input_keys,
                             output_keys)
        ## istance of the class FsmEvent
        self._trigger_event = trigger_event
        ## type to be fill in each subclass
        self.type = None
        ## publisher of topic fsm_status
        self.pub = rospy.Publisher('fsm_status', pub_status.status, queue_size=1)
        time.sleep(0.2)
        ## message status
        self.msg = pub_status.status()

    ## method action_1
    #  @param userdata 
    #  
    #  call back of the trigger "action_1"
    def action_1(self, userdata):
        ## to override
        return None

    ## method action_2
    #  @param userdata 
    #  
    #  call back of the trigger "action_2"
    def action_2(self, userdata):
        ## to override
        return None

    ## method action_3
    #  @param userdata 
    #  
    #  call back of the trigger "action_3"
    def action_3(self, userdata):
        ## to override
        return None

    ## method action_4
    #  @param userdata 
    #  
    #  call back of the trigger "action_4"
    def action_4(self, userdata):
        ## to override
        return None

    ## method action_5
    #  @param userdata 
    #  
    #  call back of the trigger "action_5"
    def action_5(self, userdata):
        ## to override
        return None

    ## method action_6
    #  @param userdata 
    #  
    #  call back of the trigger "action_6"
    def action_6(self, userdata):
        ## to override
        return None

    ## method user_detected
    #  @param userdata 
    #  
    #  call back of the trigger "user_detected"
    def user_detected(self, userdata):
        ## to override
        return None

    ## method user_left
    #  @param userdata 
    #  
    #  call back of the trigger "user_left"
    def user_left(self, userdata):
        ## to override
        return None

    ## method config
    #  @param userdata 
    #  
    #  call back of the trigger "config"
    def config(self, userdata):
        # to override 
        return None

    ## method publish_state
    #  @param userdata 
    #  
    #  publish message 
    def publish_state(self):
        # to override
        raise NotImplemented

    ## method execute
    #  @param userdata 
    #  
    #  executable code of the blocking state
    def execute(self, userdata):
        while True:
            self.publish_state()

            if self.preempt_requested():
                return 'preempted'
            event_id = self._trigger_event.wait()
            if self.preempt_requested():
                return 'preempted'

            ret = None

            if event_id == 'action_1':
                ret = self.action_1(userdata)
            elif event_id == 'action_2': 
                ret = self.action_2(userdata)
            elif event_id == 'action_3': 
                ret = self.action_3(userdata)
            elif event_id == 'action_4': 
                ret = self.action_4(userdata)
            elif event_id == 'action_5': 
                ret = self.action_5(userdata)
            elif event_id == 'action_6': 
                ret = self.action_6(userdata)
            elif event_id == 'user_detected':
                ret = self.user_detected(userdata)
            elif event_id == 'user_left':
                ret = self.user_left(userdata)
            elif event_id == 'config':
                ret = self.config(userdata)

            if ret:
                return ret

    ## method request_preempt 
    #  method called when the state is preempted 
    def request_preempt(self):
        smach.State.request_preempt(self)
        self._trigger_event.signal('preempt')
