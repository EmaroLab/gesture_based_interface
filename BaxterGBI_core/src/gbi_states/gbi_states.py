import roslib
import rospy
from BaxterGBI_core_srvs.srv import *
import BaxterGBI_input_msgs.msg as bgi_io
import smach
import threading
from collections import namedtuple
import BaxterGBI_core_msgs.msg as pub_status

ActionContext = namedtuple('ActionContext', ['action', 'status'])


class BlockingState(smach.State):
    def __init__(self, outcomes, trigger_event, output_keys=[], input_keys=[]):
        outcomes = outcomes + ['preempted']
        smach.State.__init__(self,
                             outcomes,
                             input_keys,
                             output_keys)

        self._trigger_event = trigger_event
        self.type = None
        self.context = None
        self.pub = rospy.Publisher('fsm_status', pub_status.status, queue_size=10)
        self.msg = pub_status.status()

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

    def publish_state(self):
        raise NotImplemented

    def execute(self, userdata):
        while True:
            self.publish_state()

            if self.preempt_requested():
                return 'preempted'
            event_id = self._trigger_event.wait()
            if self.preempt_requested():
                return 'preempted'

            ret = None

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

            if ret:
                return ret

    def request_preempt(self):
        smach.State.request_preempt(self)
        self._trigger_event.signal('preempt')


class MenuState(BlockingState):
    # FIXME: manage properly minimum set of output_keys and outcomes
    def __init__(self, outcomes, trigger_event, page_title, output_keys=[], input_keys=[], fixed_options=['back', 'run']):
        BlockingState.__init__(self,
                               outcomes = ['user_missed', 'selection'] + outcomes,
                               trigger_event = trigger_event,
                               output_keys=['selection'] + output_keys,
                               input_keys = input_keys)

        self.type = 'menu'
        self.title = page_title
        self.variable_options = []
        self.fixed_options = fixed_options
        self.selection = 0

    def action_1(self, userdata):
        max_selection = len(self.variable_options) + len(self.fixed_options)
        if self.selection < max_selection:
            self.selection += 1
        return None

    def action_2(self, userdata):
        if self.selection > 0:
            self.selection -= 1
        return None

    def action_3(self, userdata):
        if self.selection < len(self.variable_options):
            item = self.variable_options[self.selection]
            return self.on_variable_selection(self.selection, item, userdata)
        else:
            item = self.fixed_options[self.selection - len(self.variable_options)]
            return self.on_fixed_selection(self.selection - len(self.variable_options), item, userdata)

    def user_left(self, userdata):
        return 'user_missed'

    def execute(self, userdata):
        self.variable_options = self.update_variable_options(userdata)
        return BlockingState.execute(self, userdata)

    def publish_state(self):
        self.msg.context_type = self.type
        self.msg.m_title = self.title
        self.msg.m_options = self.variable_options
        self.msg.m_fixed_options = self.fixed_options
        self.msg.m_selection = self.selection
        rospy.loginfo(self.msg)
        self.pub.publish(self.msg)

    def update_variable_options(self, userdata):
        return []

    def on_variable_selection(self, index, item, userdata):
        userdata.selection = item
        return 'selection'

    def on_fixed_selection(self, index, item, userdata):
        return item


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
        rospy.loginfo("Received event " + self.event_id)
        self.trigger.set()


#  rostopic pub /example/1 BaxterGBI_input_msgs/signal '{header: auto, device_id: "1", device_type: "smartwatch", device_model: "Huawei watch 2", action_descr: "wrist up", confidence: 1.0}


def publish_state(context_type, context=None):
    pub = rospy.Publisher('fsm_status', pub_status.status, queue_size=10)
    msg = pub_status.status()
    msg.context_type = context_type
    print type(context)
    if type(context) is ActionContext:
        msg.pbr_action = context.action
        msg.pbr_msg = context.status
        rospy.loginfo(msg)
        pub.publish(msg)


class InitState(smach.State):
    def __init__(self):
        outcomes = list(['config_available',
                         'config_missing',
                         'preempted'])

        smach.State.__init__(
            self,
            outcomes)

    def execute(self, userdata):
        if self.preempt_requested():
            return 'preempted'
        for i in range(1, 7):
            key = "key_" + str(i) + "_topics"
            if not rospy.has_param(key) or not rospy.get_param(key):
                return 'config_missing'
        return 'config_available'


class ConfigState(smach.State):
    def __init__(self, msg_type, callback):
        outcomes = ['invalid',
                    'success',
                    'preempted']

        smach.State.__init__(
            self,
            outcomes)
        self._callback = callback
        self._msg_type = msg_type
        self.subscribers = []

    def execute(self, userdata):
        if self.preempt_requested():
            return 'preempted'

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


class WaitConfigState(BlockingState):
    def __init__(self, trigger_event):
        outcomes=['config_available',
                  'preempted']

        BlockingState.__init__(self, outcomes, trigger_event)
        self.type = 'config_wait'

    def config(self, userdata):
        return 'config_available'

    def publish_state(self):
        self.msg.context_type = self.type
        rospy.loginfo(self.msg)
        self.pub.publish(self.msg)


class WaitUserState(BlockingState):
    def __init__(self, trigger_event):
        outcomes=['reconf_requested',
                  'user_detected',
                  'preempted']

        BlockingState.__init__(self, outcomes, trigger_event)
        self.type = 'wait_user'

    def config(self, userdata):
        return 'reconf_requested'

    def user_detected(self, userdata):
        return 'user_detected'

    def publish_state(self):
        self.msg.context_type = self.type
        rospy.loginfo(self.msg)
        self.pub.publish(self.msg)


class MainMenuState(MenuState):
    def __init__(self, trigger_event):
        outcomes = ['user_missed',
                    'play_selected',
                    'record_selected',
                    'macro_selected',
                    'sequence_selected',
                    'preempted']

        MenuState.__init__(self,
                           outcomes,
                           trigger_event,
                           'Main menu',
                           fixed_options=[])

    def update_variable_options(self, userdata):
        return ['play',
                'record',
                'macro',
                'sequence']

    def on_variable_selection(self, index, item, userdata):
        return item + '_selected'

    def on_fixed_selection(self, index, item, userdata):
        raise RuntimeError  # should never be called in main menu


class PlayMenuState(MenuState):
    def __init__(self, trigger_event):
        outcomes = ['user_missed',
                    'selection',
                    'back',
                    'remove',
                    'preempted']

        MenuState.__init__(self,
                           outcomes,
                           trigger_event,
                           'Playback menu',
                           fixed_options=['back'])

    def update_variable_options(self, userdata):
        return ['demo record']  # TODO: ask PBR the list of files


class RecordMenuState(MenuState):
    def __init__(self, trigger_event):
        outcomes = ['user_missed',
                    'selection',
                    'back',
                    'remove',
                    'preempted']

        MenuState.__init__(self,
                           outcomes,
                           trigger_event,
                           'Recording menu',
                           fixed_options=['back', 'remove', 'new'])

    def update_variable_options(self, userdata):
        return ['demo record']  # TODO: ask PBR the list of files

    def on_fixed_selection(self, index, item, userdata):
        if item == 'new':
            userdata.selection = "new_name"  # TODO: generate new unique name for recording
            return 'selection'
        else:
            return item


class MacroMenuState(MenuState):
    def __init__(self, trigger_event):
        outcomes = ['user_missed',
                    'selection',
                    'play',
                    'back',
                    'preempted']

        MenuState.__init__(self,
                           outcomes,
                           trigger_event,
                           'Macro menu',
                           input_keys=['macro_idx', 'macro_filename'])

        self.macro_slots = [None, None, None, None, None]

    def update_variable_options(self, userdata):
        if userdata.macro_idx:  # TODO: check if it works
            if userdata.macro_filename:
                self.macro_slots[userdata.macro_idx] = userdata.macro_filename
            else:
                self.macro_slots[userdata.macro_idx] = None
        return self.macro_slots

    def on_variable_selection(self, index, item, userdata):
        userdata.selection = index
        return 'selected'

    # FIXME: 'play' outcome does not outputs macro configuration


class SubMacroMenuState(MenuState):
    def __init__(self, trigger_event):
        outcomes = ['user_missed',
                    'selection',
                    'back',
                    'preempted']

        MenuState.__init__(self,
                           outcomes,
                           trigger_event,
                           'Macro menu - selection',
                           input_keys=['macro_idx'],
                           fixed_options=['back', 'clean'])

    def update_variable_options(self, userdata):
        return ['demo record']  # TODO: ask PBR the list of files

    def on_fixed_selection(self, index, item, userdata):
        if item == 'clean':
            userdata.selection = ''
            return 'selection'
        else:
            return item


class SequenceMenuState(MenuState):
    def __init__(self, trigger_event):
        outcomes=['user_missed',
                  'selection',
                  'play',
                  'back',
                  'preempted']

        MenuState.__init__(self,
                           outcomes,
                           trigger_event,
                           'Sequence menu',
                           input_keys=['sequence_idx', 'sequence_filename'])

        self.sequence = [None]

    def update_variable_options(self, userdata):
        if userdata.sequence_idx:  # TODO: check if it works
            if userdata.sequence_filename:
                if not self.sequence[userdata.sequence_idx]:
                    self.sequence += None
                self.sequence[userdata.sequence_idx] = userdata.sequence_filename
            else:
                if self.sequence[userdata.sequence_idx]:
                    del self.sequence[userdata.sequence_idx]

        return self.sequence

    def on_variable_selection(self, index, item, userdata):
        userdata.selection = index
        return 'selected'

    # FIXME: 'play' outcome does not outputs sequence configuration


class SubSequenceMenuState(MenuState):
    def __init__(self, trigger_event):
        outcomes = ['user_missed',
                    'selection',
                    'back',
                    'preempted']

        MenuState.__init__(self,
                           outcomes,
                           trigger_event,
                           'Sequence menu - selection',
                           input_keys=['sequence_idx'],
                           fixed_options=['back', 'clean'])

    def update_variable_options(self, userdata):
        return ['demo record']  # TODO: ask PBR the list of files

    def on_fixed_selection(self, index, item, userdata):
        if item == 'clean':
            userdata.selection = ''
            return 'selection'
        else:
            return item


class RemoveMenuState(MenuState):
    # TODO: rewire in Play/Record Menus
    def __init__(self, trigger_event):
        outcomes = ['user_missed',
                    'back',
                    'preempted']

        MenuState.__init__(self,
                           outcomes,
                           trigger_event,
                           'Select the recording to delete',
                           input_keys=[],
                           fixed_options=['back'])

    def update_variable_options(self, userdata):
        return ['demo record']  # TODO: ask PBR the list of files


# TODO: implement ActionState and refactor following classes

class PlayState(BlockingState):
    def __init__(self, trigger_event):
        outcomes = ['done',
                    'user_missed',
                    'preempted']
        input_keys = ['filename']

        BlockingState.__init__(self,
                               outcomes,
                               trigger_event,
                               output_keys=[],
                               input_keys=input_keys)
        self.type = 'action'
        self.context = ActionContext(action='Playback mode',
                                     status='')

    def action_6(self, userdata):
        return 'done'

    def user_left(self, userdata):
        return 'user_missed'


class RecordState(BlockingState):
    def __init__(self, trigger_event):
        outcomes = ['done',
                    'user_missed',
                    'preempted']
        input_keys = ['filename']

        BlockingState.__init__(self,
                               outcomes,
                               trigger_event,
                               output_keys=[],
                               input_keys=input_keys)
        self.type = 'action'
        self.context = ActionContext(action='Playback mode',
                                     status='')

    def action_6(self, userdata):
        return 'done'

    def user_left(self, userdata):
        return 'user_missed'


class MacroState(BlockingState):
    def __init__(self, trigger_event):
        outcomes=['done',
                  'user_missed',
                  'preempted']
        input_keys=['macro_conf']

        BlockingState.__init__(self,
                               outcomes,
                               trigger_event,
                               output_keys=[],
                               input_keys=input_keys)
        self.type = 'action'
        self.context = ActionContext(action='Macro mode',
                                     status='')

    def action_6(self, userdata):
        return 'done'

    def user_left(self, userdata):
        return 'user_missed'


class SequenceState(BlockingState):
    def __init__(self, trigger_event):
        outcomes = ['done',
                    'user_missed',
                    'preempted']
        input_keys = ['sequence_conf']

        BlockingState.__init__(self,
                               outcomes,
                               trigger_event,
                               output_keys=[],
                               input_keys=input_keys)
        self.type = 'action'
        self.context = ActionContext(action='Sequence mode',
                                     status='')

    def action_6(self, userdata):
        return 'done'

    def user_left(self, userdata):
        return 'user_missed'
