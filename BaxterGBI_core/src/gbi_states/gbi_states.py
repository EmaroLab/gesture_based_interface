import roslib
import rospy
from BaxterGBI_core_srvs.srv import *
import BaxterGBI_input_msgs.msg as bgi_io
import smach
import threading
from collections import namedtuple

MenuContext=namedtuple('MenuContext', ['title','options','fixed_options','selction'])
ActionContext=namedtuple('ActionContext',['action','status'])

class FsmEvent:
    def __init__(self, fsm):
        self.trigger = threading.Event()
        self.event_id = None
        self.fsm = fsm
    def wait(self):
        self.trigger.clear()
        self.trigger.wait()
    def signal(self, event_id):
        self.event_id = event_id
        print "Received event " + self.event_id
        self.trigger.set()
"""
class FsmEvent():
    def __init__(self):
        self.trigger = threading.Event()
        self.event_id = None
"""
def handle_config():
    return True

def publish_state(type,context=None):
    pass

class InitState(smach.State):
    def __init__(self, input_keys = [],output_keys=[]):
        smach.State.__init__(
            self,
            outcomes = ['config_available','config_missing', 'preempted'],
            input_keys = input_keys,
            output_keys = output_keys)

    def execute(self, userdata):
        if self.preempt_requested(): return 'preempted'
        for i in range(1,7):
            key = "key_" + str(i) + "_topics"
            if not rospy.has_param(key) or not rospy.get_param(key):
                return 'config_missing'
        return 'config_available'

    def request_preempt(self):
        smach.State.request_preempt(self)

class ConfigState(smach.State):
    def __init__(self, msg_type, callback, input_keys = [],output_keys=[]):
        smach.State.__init__(
            self,
            outcomes = ['invalid','success', 'preempted'],
            input_keys = input_keys,
            output_keys = output_keys)
        self._callback = callback
        self._msg_type = msg_type
        self.subscribers=[]

    def execute(self, userdata):
        if self.preempt_requested(): return 'preempted'
        for i in range(1,7):
            key = "key_" + str(i) + "_topics"
            if not rospy.has_param(key) or not rospy.get_param(key):
                return 'invalid'
    
        for subscriber in self.subscribers:
            subscriber.unregister()
        self.subscribers = []

        for i in range(1,7):
            for topic in rospy.get_param("key_" + str(i) + "_topics"):
                sub = rospy.Subscriber(	topic,
                                                self._msg_type,
                                                self._callback,
                                                {"topic": topic, "code": i}
                )
                self.subscribers.append(sub)
        return 'success'

    def request_preempt(self):
        smach.State.request_preempt(self)

class WaitConfigState(smach.State):
    def __init__(self, trigger_event, input_keys = [],output_keys=[]):
        smach.State.__init__(
            self,
            outcomes = ['config_available', 'preempted'],
            input_keys = input_keys,
            output_keys = output_keys)

        self._trigger_event = trigger_event
        self.type='config_wait'

    def execute(self, userdata):
        publish_state(self.type)
        while True:
            if self.preempt_requested(): return 'preempted'
            self._trigger_event.wait()
            if self.preempt_requested(): return 'preempted'
            if self._trigger_event.event_id == 'config':
                return 'config_available'

    def request_preempt(self):
        smach.State.request_preempt(self)
        self._trigger_event.signal('preempt')


class WaitUserState(smach.State):
    def __init__(self, trigger_event ,input_keys = [],output_keys=[]):
        smach.State.__init__(
            self,
            outcomes = ['reconf_requested','user_detected', 'preempted'],
            input_keys = input_keys,
            output_keys = output_keys)

        self._trigger_event = trigger_event
        self.type='wait_user'

    def execute(self, userdata):
        publish_state(self.type)
        while True:
            if self.preempt_requested(): return 'preempted'
            self._trigger_event.wait()
            if self.preempt_requested(): return 'preempted'
            if self._trigger_event.event_id == 'user_detected':
                return 'user_dected'
            elif self._trigger_event.event_id == 'config':
                return 'reconf_requested'

    def request_preempt(self):
        smach.State.request_preempt(self)
        self._trigger_event.signal('preempt')

class MenuState(smach.State):
    def __init__(self, trigger_event, input_keys = [],output_keys=[]):
        smach.State.__init__(
            self,
            outcomes = ['user_missed','play_selected','record_selected',
                       'macro_selected','sequence_selected', 'preempted'],
            input_keys = input_keys,
            output_keys = output_keys)

        self._trigger_event = trigger_event
        self.type='menu'

    def execute(self, userdata):
        publish_state(self.type)
        ''' choose from the possible option of the user and the return the proper outcome'''
        if self.preempt_requested(): return 'preempted'
        self._trigger_event.wait()
        if self.preempt_requested(): return 'preempted'
        if self._trigger_event.event_id == 'action_1' #up
            ''' move up'''
        elif self._trigger_event.event_id == 'action_2' # down
            # move down
        elif self._trigger_event.event_id == 'action_3' # select
            ''' context status '''
            return  # status 
        elif self._trigger_event.event_id == 'user_missed'
            return 'user_missed'

    def request_preempt(self):
        smach.State.request_preempt(self)
        self._trigger_event.signal('preempt')

class PlayState(smach.State):
    def __init__(self, trigger_event, input_keys = [],output_keys=[]):
        smach.State.__init__(
            self,
            outcomes = ['done', 'preempted'],
            input_keys = input_keys,
            output_keys = output_keys)

        self._trigger_event = trigger_event
        self.type='play'

    def execute(self, userdata):
        publish_state(self.type)
        '''choose the file to execute and wait for the signal
        send a message to control part and wait for the message 
        of the end of operation'''

    def request_preempt(self):
        smach.State.request_preempt(self)
        self._trigger_event.signal('preempt')

class RecordState(smach.State):
    def __init__(self, trigger_event, input_keys = [],output_keys=[]):
        smach.State.__init__(
            self,
            outcomes = ['done', 'preempted'],
            input_keys = input_keys,
            output_keys = output_keys)

        self._trigger_event = trigger_event
        self.type='record'

    def execute(self, userdata):
        publish_state(self.type)
        '''send a message to control part and wait for the message 
        of the end of operation'''

    def request_preempt(self):
        smach.State.request_preempt(self)
        self._trigger_event.signal('preempt')

class MacroState(smach.State):
    def __init__(self, trigger_event, input_keys = [],output_keys=[]):
        smach.State.__init__(
            self,
            outcomes = ['done', 'preempted'],
            input_keys = input_keys,
            output_keys = output_keys)

        self._trigger_event = trigger_event
        self.type='macro'

    def execute(self, userdata):
        publish_state(self.type)
        '''choose the file to execute and wait for the signal
        send a message to control part and wait for the message 
        of the end of operation'''

    def request_preempt(self):
        smach.State.request_preempt(self)
        self._trigger_event.signal('preempt')

class SequenceState(smach.State):
    def __init__(self, trigger_event, input_keys = [],output_keys=[]):
        smach.State.__init__(
            self,
            outcomes = ['done', 'preempted'],
            input_keys = input_keys,
            output_keys = output_keys)

        self._trigger_event = trigger_event
        self.type='sequence'

    def execute(self, userdata):
        publish_state(self.type)
        '''choose the file to execute and wait for the signal
        if the event id is play send a message to control part and wait for the message 
        of the end of operation'''

    def request_preempt(self):
        smach.State.request_preempt(self)
        self._trigger_event.signal('preempt')

