import rospy
import threading

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
