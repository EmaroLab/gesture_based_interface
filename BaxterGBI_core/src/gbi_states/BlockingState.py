import rospy
import smach
import BaxterGBI_core_msgs.msg as pub_status


class BlockingState(smach.State):
    def __init__(self, outcomes, trigger_event, output_keys=[], input_keys=[]):
        outcomes = outcomes + ['preempted']
        smach.State.__init__(self,
                             outcomes,
                             input_keys,
                             output_keys)

        self._trigger_event = trigger_event
        self.type = None
        self.timeout=5
        self.timeout_t=-1
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
            elif self.timeout>=0 && time.time() > self.timeout:
                ret = self.user_left(userdata)
            elif event_id == 'config':
                ret = self.config(userdata)

            if ret:
                return ret

    def request_preempt(self):
        smach.State.request_preempt(self)
        self._trigger_event.signal('preempt')
