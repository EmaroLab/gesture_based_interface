# -*- coding: latin-1 -*-
## @package BlockingState
## This package describes the general blocking state

import rospy
import smach
import baxter_gbi_core_msgs.msg as pub_status

## BlockingState
# inherited form smach.State
class BlockingState(smach.State):
    ## constructor
    # @param outcomes possible outcomes of the state
    # @param trigger_event object of the class FsmEvent
    # @param output_keys set of the data in output
    # @param input_keys set of the data in input
    def __init__(self, outcomes, trigger_event, output_keys=[], input_keys=[]):
        outcomes = outcomes + ['preempted']
        smach.State.__init__(self,
                             outcomes,
                             input_keys,
                             output_keys)
        ## instance of the class FsmEvent
        self._trigger_event = trigger_event
        ## type to be fill in each subclass
        self.type = None
        ## publisher of topic fsm_status
        self.pub = rospy.Publisher('fsm_status', pub_status.status, queue_size=1, latch=True)
        ## message status
        self.msg = pub_status.status()
        self.running = False
        self.status_update_inhibited = False

    ## method action_1
    # call back of the trigger "action_1"
    # 
    # @param userdata
    def action_1(self, userdata):
        ## to override
        return None

    ## method action_2
    # call back of the trigger "action_2"
    #  
    # @param userdata
    def action_2(self, userdata):
        ## to override
        return None

    ## method action_3
    # callback of the trigger "action_3"
    #  
    # @param userdata
    def action_3(self, userdata):
        ## to override
        return None

    ## method action_4
    # callback of the trigger "action_4"
    #  
    # @param userdata
    def action_4(self, userdata):
        ## to override
        return None

    ## method action_5
    # call back of the trigger "action_5"
    #  
    # @param userdata
    def action_5(self, userdata):
        ## to override
        return None

    ## method action_6
    # call back of the trigger "action_6"
    #  
    # @param userdata
    def action_6(self, userdata):
        ## to override
        return None

    ## method user_detected
    # callback of the trigger "user_detected"
    #  
    # @param userdata
    def user_detected(self, userdata):
        ## to override
        return None

    ## method user_left
    # @param userdata
    #  
    # callback of the trigger "user_left"
    def user_left(self, userdata):
        ## to override
        return None

    ## method config
    # call back of the trigger "config"
    #  
    # @param userdata
    def config(self, userdata):
        # to override 
        return None

    ## method done
    # @param userdata
    def done(self,userdata):
        return None

    ## method publish_state
    # publish message
    #  
    # @param userdata
    def publish_state(self):
        # to override
        raise NotImplemented

    ## method execute
    # executable code of the blocking state
    #  
    # @param userdata
    def execute(self, userdata):
        self.running = True
        while True:
            if not self.status_update_inhibited:
                self.publish_state()
                self.status_update_inhibited = False

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
            elif event_id == 'finished':
                ret = self.done(userdata)

            if ret:
                self.running = False
                return ret

    ## method request_preempt
    #
    # method called when the state is preempted
    def request_preempt(self):
        smach.State.request_preempt(self)
        self._trigger_event.signal('preempt')

    ## method signal
    # signals the event with the corresponding id
    #
    # @param event id id of the event
    def signal(self, event_id):
        self._trigger_event.signal(event_id)

    ## method is_running
    def is_running(self):
        return self.running

    def inhibit_update(self):
        self.status_update_inhibited = True
