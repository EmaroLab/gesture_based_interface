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