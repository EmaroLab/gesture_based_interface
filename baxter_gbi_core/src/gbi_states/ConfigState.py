# -*- coding: latin-1 -*-
## @package ConfigState
#  The package defines the structure of the configuration state

import rospy
import smach
import baxter_gbi_input_msgs.msg as bgi_io

## ConfigState
# outcomes: (invalid,success,'preempted')
class ConfigState(smach.State):
    ## constructor
    # @param msg_type
    # @param callback
    def __init__(self, trigger_event):
        outcomes = ['invalid',
                    'success',
                    'preempted']

        self._trigger_event = trigger_event
        smach.State.__init__(
            self,
            outcomes)
        self.subscribers = []

    def action_cb(self, msg, params):
        self._trigger_event.signal('action_' + str(params["code"]))

    ## method execute
    # @param userdata
    #
    # executes the configuration state
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
                                       bgi_io.signal,
                                       self.action_cb,
                                       {"topic": topic, "code": i}
                                       )
                self.subscribers.append(sub)
        return 'success'
