import roslib
import rospy
from BaxterGBI_core_srvs.srv import *
import BaxterGBI_input_msgs.msg as bgi_io
import smach
import threading
from collections import namedtuple
import BaxterGBI_core_msgs.msg as pub_status

MenuContext=namedtuple('MenuContext', ['title','options','fixed_options','selection'])
ActionContext=namedtuple('ActionContext',['action','status'])

play_option=[]
macro_option=[]

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

#  rostopic pub /example/1 BaxterGBI_input_msgs/signal '{header: auto, device_id: "1", device_type: "smartwatch", device_model: "Huawei watch 2", action_descr: "wrist up", confidence: 1.0}

def handle_config():
    return True

pub = rospy.Publisher('fsm_status', pub_status.status, queue_size=10)
def publish_state(context_type,context=None):
    global pub
    msg = pub_status.status()
    msg.context_type=context_type
    if type(context) is MenuContext:
        msg.m_title=context.title
        msg.m_options=context.options
        msg.m_fixed_options=context.fixed_options
        msg.m_selection=context.selection
    if type(context) is ActionContext:
        msg.pbr_action=context.action
        msg.pbr_msg=context.status
    rospy.loginfo(msg)
    pub.publish(msg)

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
        publish_state(self.type,MenuContext(title='lalalala',options='1',fixed_options='state_conf',selection=''))
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
        self.status=MenuContext(title='Menu',options=['play_selected','record_selected',
                             'macro_selected','sequence_selected'],fixed_options=[],selection=0)
        self.options = play_option + self.status.fixed_options
        self.MaxSelection=len(self.options)

    def execute(self, userdata):
        ''' choose from the possible option of the user and the return the proper outcome'''
        while(1):
            publish_state(self.type,self.status)
            if self.preempt_requested(): return 'preempted'
            self._trigger_event.wait()
            if self.preempt_requested(): return 'preempted'
            if self._trigger_event.event_id == 'action_1': #up
                if self.status.selection==self.MaxSelection : self.status.selection=self.MaxSelection 
                else: self.status.selection=self.status.selection+1
            elif self._trigger_event.event_id == 'action_2': # down
                if self.status.selection==0 : self.status.selection=0 
                else: self.status.selection=self.status.selection-1
            elif self._trigger_event.event_id == 'action_3': # select
                return self.options[self.status.selection]
            elif self._trigger_event.event_id == 'user_missed':
                 return 'user_missed'


    def request_preempt(self):
        smach.State.request_preempt(self)
        self._trigger_event.signal('preempt')

class PlayMenuState(smach.State):
    def __init__(self, trigger_event, input_keys = [],output_keys=[]):
        smach.State.__init__(
            self,
            outcomes = ['user_missed','play_selected','sequence_selected','back',
                        'preempted'],
            input_keys = input_keys,
            output_keys = output_keys)

        self._trigger_event = trigger_event
        self.type='menu'
        self.status=MenuContext(title='Menu',options=play_option,fixed_options=['back','sequence'],selection=0)
        self.options = play_option + self.status.fixed_options
        self.MaxSelection=len(self.options)

    def execute(self, userdata):
        ''' choose from the possible option of the user and the return the proper outcome'''
        while(1):
            publish_state(self.type,self.status)
            if self.preempt_requested(): return 'preempted'
            self._trigger_event.wait()
            if self.preempt_requested(): return 'preempted'
            if self._trigger_event.event_id == 'action_2': #up
                if self.status.selection==self.MaxSelection : self.status.selection=self.MaxSelection 
                else: self.status.selection=self.status.selection+1
            elif self._trigger_event.event_id == 'action_1': # down
                if self.status.selection==0 : self.status.selection=0 
                else: self.status.selection=self.status.selection-1
            elif self._trigger_event.event_id == 'action_3': # select
                if self.options[self.status.selection]=='back': return 'back'
                if self.options[self.status.selection]=='sequence': return 'sequence_selected'
                publish_state(self.type,ActionContext(
                    action=self.status.options[self.status.selection]
                    ,status=''))
                return 'play_selected'
            elif self._trigger_event.event_id == 'user_missed':
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
        self.status=MenuContext(title='play',options=play_option,fixed_options=['back'],selection=0)
        self.options = play_option + self.status.fixed_options
        self.MaxSelection=len(self.options)

    def execute(self, userdata):
        publish_state(self.type)
        if self.preempt_requested(): return 'preempted'
        '''choose the file to execute and wait for the signal
        send a message to control part and wait for the message 
        of the end of operation'''
        return 'done'

    def request_preempt(self):
        smach.State.request_preempt(self)
        self._trigger_event.signal('preempt')

class RecordMenuState(smach.State):
    def __init__(self, trigger_event, input_keys = [],output_keys=[]):
        smach.State.__init__(
            self,
            outcomes = ['user_missed','record_selected','back','remove','preempted'],
            input_keys = input_keys,
            output_keys = output_keys)

        self._trigger_event = trigger_event
        self.type='menu'
        self.status=MenuContext(title='Menu',options=play_option,fixed_options=['add_new','remove,back'],selection=0)
        self.options = play_option + self.status.fixed_options
        self.MaxSelection=len(self.options)

    def execute(self, userdata):
        ''' choose from the possible option of the user and the return the proper outcome'''
        while(1):
            publish_state(self.type,self.status)
            if self.preempt_requested(): return 'preempted'
            self._trigger_event.wait()
            if self.preempt_requested(): return 'preempted'
            if self._trigger_event.event_id == 'action_1': #up
                if self.status.selection==self.MaxSelection : self.status.selection=self.MaxSelection 
                else: self.status.selection=self.status.selection+1
            elif self._trigger_event.event_id == 'action_2': # down
                if self.status.selection==0 : self.status.selection=0 
                else: self.status.selection=self.status.selection-1
            elif self._trigger_event.event_id == 'action_3': # select
                if self.options[self.status.selection]=='back': return 'back'
                if self.options[self.status.selection]=='remove': return 'remove'
                if self.options[self.status.selection]=='add_new':return 'record_selected'
            elif self._trigger_event.event_id == 'user_missed':
                 return 'user_missed'



    def request_preempt(self):
        smach.State.request_preempt(self)
        self._trigger_event.signal('preempt')

class RemoveState(smach.State):
    def __init__(self, trigger_event, input_keys = [],output_keys=[]):
        smach.State.__init__(
            self,
            outcomes = ['done','user_missed','preempted'],
            input_keys = input_keys,
            output_keys = output_keys)

        self._trigger_event = trigger_event
        self.type='RemoveState'
        self.status=MenuContext(title='Remove',options=play_option,fixed_options=['back'],selection=0)
        self.options = play_option + self.status.fixed_options
        self.MaxSelection=len(self.options)

    def execute(self, userdata):
        if self.preempt_requested(): return 'preempted'
        while(1):
            publish_state(self.type,self.status)
            if self.preempt_requested(): return 'preempted'
            self._trigger_event.wait()
            if self.preempt_requested(): return 'preempted'
            if self._trigger_event.event_id == 'action_1': #up
                if self.status.selection==self.MaxSelection : self.status.selection=self.MaxSelection 
                else: self.status.selection=self.status.selection+1
            elif self._trigger_event.event_id == 'action_2': # down
                if self.status.selection==0 : self.status.selection=0 
                else: self.status.selection=self.status.selection-1
            elif self._trigger_event.event_id == 'action_3': # select
                if self.options[self.status.selection]=='back': return 'done'
                play_option.remove[self.options[self.status.selection]]
                self.status.options=play_option
                self.options = play_option + fixed_options
                self.MaxSelection=len(self.option)
            elif self._trigger_event.event_id == 'user_missed':
                 return 'user_missed'

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
        if self.preempt_requested(): return 'preempted'
        '''send a message to control part and wait for the message 
        of the end of operation'''
        return 'done'

    def request_preempt(self):
        smach.State.request_preempt(self)
        self._trigger_event.signal('preempt')

class MacroMenuState(smach.State):
    def __init__(self, trigger_event, input_keys = [],output_keys=[]):
        smach.State.__init__(
            self,
            outcomes = ['user_missed','macro_selected','back',
                        'preempted'],
            input_keys = input_keys,
            output_keys = output_keys)

        self._trigger_event = trigger_event
        self.type='menu'
        self.status=MenuContext(title='MacroMenu',options=play_option,fixed_options=['back'],selection=0)
        self.options = play_option + self.status.fixed_options
        self.MaxSelection=len(self.options)

    def execute(self, userdata):
        if self.preempt_requested(): return 'preempted'
        while(1):
            publish_state(self.type,self.status)
            if self.preempt_requested(): return 'preempted'
            self._trigger_event.wait()
            if self.preempt_requested(): return 'preempted'
            if self._trigger_event.event_id == 'action_1': #up
                if self.status.selection==self.MaxSelection : self.status.selection=self.MaxSelection 
                else: self.status.selection=self.status.selection+1
            elif self._trigger_event.event_id == 'action_2': # down
                if self.status.selection==0 : self.status.selection=0 
                else: self.status.selection=self.status.selection-1
            elif self._trigger_event.event_id == 'action_3': # select
                if self.options[self.status.selection]=='back': return 'back'
                # pass the macro to the following state
                return 'macro_selected'
            elif self._trigger_event.event_id == 'user_missed':
                 return 'user_missed'

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
        if self.preempt_requested(): return 'preempted'
        '''choose the file to execute and wait for the signal
        send a message to control part and wait for the message 
        of the end of operation'''
        return 'done'

    def request_preempt(self):
        smach.State.request_preempt(self)
        self._trigger_event.signal('preempt')

class SequenceMenuState(smach.State):
    def __init__(self, trigger_event, input_keys = [],output_keys=[]):
        smach.State.__init__(
            self,
            outcomes = ['user_missed','sequence_selected','back','preempted'],
            input_keys = input_keys,
            output_keys = output_keys)

        self._trigger_event = trigger_event
        self.type='menu'
        self.status=MenuContext(title='SequenceMenu',options=play_option,fixed_options=['back', 'play'],selection=0)
        self.options = play_option + self.status.fixed_options
        self.MaxSelection=len(self.options)
        self.selected_option=[]

    def execute(self, userdata):
        if self.preempt_requested(): return 'preempted'
        while(1):
            publish_state(self.type,self.status)
            if self.preempt_requested(): return 'preempted'
            self._trigger_event.wait()
            if self.preempt_requested(): return 'preempted'
            if self._trigger_event.event_id == 'action_1': #up
                if self.status.selection==self.MaxSelection : self.status.selection=self.MaxSelection 
                else: self.status.selection=self.status.selection+1
            elif self._trigger_event.event_id == 'action_2': # down
                if self.status.selection==0 : self.status.selection=0 
                else: self.status.selection=self.status.selection-1
            elif self._trigger_event.event_id == 'action_3': # select
                if self.options[self.status.selection]=='back': return 'back'
                if self.options[self.status.selection]=='play':
                    return 'sequence_selected' #pass the selected_option to the following state 
                self.selected_option=self.selected_option+self.options[self.status.selection]
            elif self._trigger_event.event_id == 'user_missed':
                 return 'user_missed'

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
        if self.preempt_requested(): return 'preempted'
        '''choose the file to execute and wait for the signal
        if the event id is play send a message to control part and wait for the message 
        of the end of operation'''
        return 'done'

    def request_preempt(self):
        smach.State.request_preempt(self)
        self._trigger_event.signal('preempt')

