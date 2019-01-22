import roslib
import rospy
from BaxterGBI_core_srvs.srv import *
import BaxterGBI_input_msgs.msg as bgi_io
import smach
import threading
from collections import namedtuple
import BaxterGBI_core_msgs.msg as pub_status

MenuContext = namedtuple('MenuContext', ['title', 'options', 'fixed_options', 'selection'])
ActionContext = namedtuple('ActionContext', ['action', 'status'])

play_option = []
macro_option = []


class BlockingState(smach.State):
    def __init__(self, trigger_event, outcomes,output_keys=[],input_keys=[]):
        smach.State.__init__(self,
                            outcomes,
                            input_keys,
                            output_keys)

        self._trigger_event = trigger_event
        self.type = None
        self.context = None

    def action_1(self, userdata):
        return None

    def action_2(self, userdata):
        return None

    def action_3(self, userdata):
        return None

    def action_4(self, userdata):
        return None

    def action_5(self, userdata):
        return None

    def action_6(self, userdata):
        return None

    def user_detected(self, userdata):
        return None

    def user_left(self, userdata):
        return None

    def config(self, userdata):
        return None

    def execute(self, userdata):
        while True:
            publish_state(self.type, self.context)

            if self.preempt_requested(): return 'preempted'
            event_id = self._trigger_event.wait()
            if self.preempt_requested(): return 'preempted'

            if event_id == 'action_1':  # up
                ret = self.action_1(userdata)
            elif event_id == 'action_2':  # up
                ret = self.action_2(userdata)
            elif event_id == 'action_3':  # up
                ret = self.action_3(userdata)
            elif event_id == 'action_4':  # up
                ret = self.action_4(userdata)
            elif event_id == 'action_5':  # up
                ret = self.action_5(userdata)
            elif event_id == 'action_6':  # up
                ret = self.action_6(userdata)
            elif event_id == 'user_detected':  # up
                ret = self.user_detected(userdata)
            elif event_id == 'user_left':  # up
                ret = self.user_left(userdata)
            elif event_id == 'config':  # up
                ret = self.config(userdata)
            if ret != None: return ret

    def request_preempt(self):
        smach.State.request_preempt(self)
        self._trigger_event.signal('preempt')


class FsmEvent:
    def __init__(self, fsm):
        self.trigger = threading.Event()
        self.event_id = None
        self.fsm = fsm

    def wait(self):
        self.trigger.clear()
        self.trigger.wait()
        return self.event_id

    def signal(self, event_id):
        self.event_id = event_id
        print "Received event " + self.event_id
        self.trigger.set()


#  rostopic pub /example/1 BaxterGBI_input_msgs/signal '{header: auto, device_id: "1", device_type: "smartwatch", device_model: "Huawei watch 2", action_descr: "wrist up", confidence: 1.0}


pub = rospy.Publisher('fsm_status', pub_status.status, queue_size=10)
pub1 = rospy.Publisher('sm_action', pub_status.status, queue_size=10)


def publish_state(context_type, context=None):
    global pub
    global pub1
    msg = pub_status.status()
    msg.context_type = context_type
    print type(context)
    if type(context) is MenuContext:
        msg.m_title = context.title
        msg.m_options = context.options
        msg.m_fixed_options = context.fixed_options
        msg.m_selection = context.selection
        rospy.loginfo(msg)
        pub.publish(msg)
    elif type(context) is ActionContext:
        msg.pbr_action = context.action
        msg.pbr_msg = context.status
        rospy.loginfo(msg)
        pub1.publish(msg)

################################################################

class InitState(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['config_available', 'config_missing', 'preempted'])

    def execute(self, userdata):
        if self.preempt_requested(): return 'preempted'
        for i in range(1, 7):
            key = "key_" + str(i) + "_topics"
            if not rospy.has_param(key) or not rospy.get_param(key):
                return 'config_missing'
        return 'config_available'

################################################################

class ConfigState(smach.State):
    def __init__(self, msg_type, callback):
        smach.State.__init__(
            self,
            outcomes=['invalid',
                      'success',
                      'preempted'])
        self._callback = callback
        self._msg_type = msg_type
        self.subscribers = []

    def execute(self, userdata):
        if self.preempt_requested(): return 'preempted'

        for i in range(1, 7):
            key = "key_" + str(i) + "_topics"
            if not rospy.has_param(key) or not rospy.get_param(key):
                return 'invalid'

        for subscriber in self.subscribers:
            subscriber.unregister()
        self.subscribers = []

        for i in range(1, 7):
            for topic in rospy.get_param("key_" + str(i) + "_topics"):
                sub = rospy.Subscriber(topic,
                                       self._msg_type,
                                       self._callback,
                                       {"topic": topic, "code": i}
                                       )
                self.subscribers.append(sub)
        return 'success'

################################################################

class WaitConfigState(BlockingState):
    def __init__(self, trigger_event):
        outcomes=['config_available',
                  'preempted']

        BlockingState.__init__(self, trigger_event, outcomes)
        self.type = 'config_wait'
        self.context = MenuContext(title='ConfigState',
                                   options=[],
                                   fixed_options=[],
                                   selection=0)
    

    def config(self, userdata):
        return 'config_available'

################################################################

class WaitUserState(BlockingState):
    def __init__(self, trigger_event):
        outcomes=['reconf_requested',
                      'user_detected',
                      'preempted']

        BlockingState.__init__(self, trigger_event, outcomes)
        self.type = 'wait_user'

    def config(self, userdata):
        return 'reconf_requested'
    def user_detected(self, userdata):
        return 'user_detected'

################################################################

class MenuState(BlockingState):
    def __init__(self, trigger_event):
        outcomes=['user_missed',
                    'play_selected',
                    'record_selected',
                    'macro_selected',
                    'sequence_selected',
                    'preempted']

        BlockingState.__init__(self, trigger_event, outcomes)
        self.type = 'menu'
        self.context = MenuContext(title='Menu',
                                   options=['play',
                                            'record',
                                            'macro',
                                            'sequence'],
                                   fixed_options=[],
                                   selection=0)
        self.options = ['play_selected','record_selected',
                        'macro_selected','sequence_selected'] + self.context.fixed_options
        self.MaxSelection = len(self.options)

    def action_1(self,userdata):
        if self.context.selection < self.MaxSelection:
            self.context.selection += 1
        return None

    def action_2(self,userdata):
        if self.context.selection > 0:
            self.context.selection -= 1
        return None

    def action_3(self,userdata):
        return self.options[self.context.selection]

    def user_left(self, userdata):
        return 'user_missed'

################################################################

class PlayMenuState(BlockingState):
    def __init__(self, trigger_event):
        outcomes=['user_missed',
                'play_selected',
                'sequence_selected',
                'back',
                'preempted']
        output_keys=['filename']

        BlockingState.__init__(self, trigger_event, outcomes, output_keys)
        self.type = 'menu'
        self.context = MenuContext(title='PlayMenu',
                                   options=play_option,
                                   fixed_options=['back',
                                                  'sequence'],
                                   selection=0)
        self.options = play_option + self.context.fixed_options
        self.MaxSelection = len(self.options)

    def action_1(self,userdata):
        if self.context.selection < self.MaxSelection:
            self.context.selection += 1
        return None

    def action_2(self,userdata):
        if self.context.selection > 0:
            self.context.selection -= 1
        return None

    def action_3(self,userdata):
        if self.options[self.context.selection] == 'back':
            return 'back'
        elif self.options[self.context.selection] == 'sequence':
            return 'sequence_selected'
        else:
            userdata.filename = self.options[self.context.selection]
            return 'play_selected'

    def user_left(self, userdata):
        return 'user_missed'

################################################################

class PlayState(smach.State):
    def __init__(self, trigger_event):
        smach.State.__init__(
            self,
            outcomes=['done',
                      'preempted'],
            input_keys=['filename'])

        self._trigger_event = trigger_event
        self.type = 'action'
        self.context = ActionContext(action='Play',
                                     status='Playing unknown file')

    def execute(self, userdata):
        while True:
            self.context.status = "Playing " + userdata.filename
            publish_state(self.type, self.context)

            if self.preempt_requested(): return 'preempted'
            self._trigger_event.wait()
            if self.preempt_requested(): return 'preempted'

    def request_preempt(self):
        smach.State.request_preempt(self)
        self._trigger_event.signal('preempt')

################################################################

class RecordMenuState(BlockingState):
    def __init__(self, trigger_event):
        outcomes=['user_missed',
                  'record_selected',
                  'back',
                  'remove',
                  'preempted']
        output_keys=['filename']

        BlockingState.__init__(self, trigger_event, outcomes,output_keys)
        self.type = 'menu'
        self.context = MenuContext(title='Record menu',
                                   options=play_option,
                                   fixed_options=['add_new',
                                                  'remove',
                                                  'back'],
                                   selection=0)
        self.options = play_option + self.context.fixed_options
        self.MaxSelection = len(self.options)
    
    def action_1(self,userdata):
        if self.context.selection < self.MaxSelection:
            self.context.selection += 1
        return None

    def action_2(self,userdata):
        if self.context.selection > 0:
            self.context.selection -= 1
        return None

    def action_3(self,userdata):
        if self.options[self.context.selection] == 'back': return 'back'
        if self.options[self.context.selection] == 'remove': return 'remove'
        if self.options[self.context.selection] == 'add_new': return 'record_selected'
        else:
            userdata.filename = self.options[self.context.selection]
            return 'record_selected'

    def user_left(self, userdata):
        return 'user_missed'

################################################################

class RecordState(smach.State):
    def __init__(self, trigger_event):
        smach.State.__init__(
            self,
            outcomes=['done', 'preempted'],
            input_keys=['filename'])

        self._trigger_event = trigger_event
        self.type = 'record'

    def execute(self, userdata):
        publish_state(self.type)
        if self.preempt_requested(): return 'preempted'
        '''send a message to control part and wait for the message 
        of the end of operation'''
        return 'done'

    def request_preempt(self):
        smach.State.request_preempt(self)
        self._trigger_event.signal('preempt')
################################################################

class MacroMenuState(BlockingState):
    def __init__(self, trigger_event):
        
        outcomes=['user_missed',
                'macro_selected',
                'play',
                'back',
                'preempted']
        output_keys=['macro_conf']

        BlockingState.__init__(self, trigger_event, outcomes,output_keys)
        self.type = 'menu'
        self.context = MenuContext(title='Macro Menu',
                                   options=[None, None, None, None, None],
                                   fixed_options=['back','play'],
                                   selection=0)
        self.options = macro_option + self.context.fixed_options
        self.MaxSelection = len(self.options)

    def action_1(self,userdata):
        if self.context.selection < self.MaxSelection:
            self.context.selection += 1
        return None

    def action_2(self,userdata):
        if self.context.selection > 0:
            self.context.selection -= 1
        return None

    def action_3(self,userdata):
        if self.options[self.context.selection] == 'back': return 'back'
        if self.options[self.context.selection] == 'play': return 'play'
        # open new menu to select the action
        return 'macro_selected'

    def user_left(self, userdata):
        return 'user_missed'

################################################################

class SubMacroMenuState(BlockingState):
    def __init__(self, trigger_event):
        outcomes=['user_missed',
                'macro_selected',
                'back',
                'preempted']
        output_keys=['macro_conf']

        BlockingState.__init__(self, trigger_event, outcomes,output_keys)
        self.type = 'menu'
        self.context = MenuContext(title='ChooseMenu',
                                   options=play_option,
                                   fixed_options=['back'],
                                   selection=0)
        self.options = macro_option + self.context.fixed_options
        self.MaxSelection = len(self.options)

    def action_1(self,userdata):
        if self.context.selection < self.MaxSelection:
            self.context.selection += 1
        return None
    def action_2(self,userdata):
        if self.context.selection > 0:
            self.context.selection -= 1
        return None

    def action_3(self,userdata):
        if self.options[self.context.selection] == 'back': return 'back'
        # open new menu to select the action
        return 'macro_selected'

    def user_left(self, userdata):
        return 'user_missed'

################################################################

class MacroState(BlockingState):
    def __init__(self, trigger_event):
        outcomes=['done','user_missed',
                'preempted']
        input_keys=['macro_conf']

        BlockingState.__init__(self, trigger_event, outcomes,[],input_keys)
        self.type = 'action'
        self.context = ActionContext(action='Macro mode',
                                     status='')

    def action_1(self, userdata):
        return None

    def action_2(self, userdata):
        return None

    def action_3(self, userdata):
        return None

    def action_4(self, userdata):
        return None

    def action_5(self, userdata):
        return None

    def action_6(self, userdata):
        return 'done'

    def user_left(self, userdata):
        return 'user_missed'

################################################################

class SequenceMenuState(BlockingState):
    def __init__(self, trigger_event, input_keys=[], output_keys=[]):
        outcomes=['user_missed', 'sequence_selected', 'back', 'preempted']
        input_keys=input_keys
        output_keys=output_keys

        BlockingState.__init__(self, trigger_event, outcomes,output_keys,input_keys)
        self.type = 'menu'
        self.context = MenuContext(title='SequenceMenu', options=[''], fixed_options=['back', 'play'],
                                   selection=0)
        self.options = play_option + self.context.fixed_options
        self.MaxSelection = len(self.options)
        self.selected_option = []

    def action_1(self,userdata):
        if self.context.selection < self.MaxSelection:
            self.context.selection += 1
        return None

    def action_2(self,userdata):
        if self.context.selection > 0:
            self.context.selection -= 1
        return None

    def action_3(self,userdata):
        if self.options[self.context.selection] == 'back': return 'back'
        if self.options[self.context.selection] == 'play':
            return 'sequence_selected'  # pass the selected_option to the following state
        self.selected_option = self.selected_option + self.options[self.context.selection]
        return None

    def user_left(self, userdata):
        return 'user_missed'

class SubSequenceMenuState(BlockingState):
    def __init__(self, trigger_event):
        outcomes=['user_missed',
                'selected',
                'back',
                'preempted']
        output_keys=['macro_conf']

        BlockingState.__init__(self, trigger_event, outcomes,output_keys)
        self.type = 'menu'
        self.context = MenuContext(title='ChooseMenu',
                                   options=play_option,
                                   fixed_options=['back'],
                                   selection=0)
        self.options = macro_option + self.context.fixed_options
        self.MaxSelection = len(self.options)

    def action_1(self,userdata):
        if self.context.selection < self.MaxSelection:
            self.context.selection += 1
        return None
    def action_2(self,userdata):
        if self.context.selection > 0:
            self.context.selection -= 1
        return None

    def action_3(self,userdata):
        if self.options[self.context.selection] == 'back': return 'back'
        # open new menu to select the action
        return 'selected'

    def user_left(self, userdata):
        return 'user_missed'

################################################################

class SequenceState(BlockingState):
    def __init__(self, trigger_event, input_keys=[], output_keys=[]):
        smach.State.__init__(
            self,
            outcomes=['done', 'preempted'],
            input_keys=input_keys,
            output_keys=output_keys)

        self._trigger_event = trigger_event
        self.type = 'sequence'

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

################################################################

class RemoveState(BlockingState):
    def __init__(self, trigger_event):
        outcomes=['done', 'user_missed', 'preempted']

        BlockingState.__init__(self, trigger_event, outcomes)
        self.type = 'RemoveState'
        self.context = MenuContext(title='Remove',
                                   options=play_option,
                                   fixed_options=['back'],
                                   selection=0)
        self.options = play_option + self.context.fixed_options
        self.MaxSelection = len(self.options)

    def action_1(self,userdata):
        if self.context.selection < self.MaxSelection:
            self.context.selection += 1
        return None

    def action_2(self,userdata):
        if self.context.selection > 0:
            self.context.selection -= 1
        return None

    def action_3(self,userdata):
        if self.options[self.context.selection] == 'back': return 'done'
        play_option.remove[self.options[self.context.selection]]
        self.context.options = play_option
        self.options = play_option + fixed_options
        self.MaxSelection = len(self.option)
        return None

    def user_left(self, userdata):
        return 'user_missed'
