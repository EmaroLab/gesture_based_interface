import rospy
import smach

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