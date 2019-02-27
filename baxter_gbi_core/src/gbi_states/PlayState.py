# -*- coding: latin-1 -*-
from ActionState import ActionState
import rospy


class PlayState(ActionState):
    def __init__(self, trigger_event):
        input_keys = ['filename', 'resume']

        ActionState.__init__(self,
                             outcomes=["pause"],
                             trigger_event=trigger_event,
                             status='play',
                             output_keys=[],
                             input_keys=input_keys)

    def cb_done(self, status, result):
        ActionState.end = True
        if not ActionState.killing:
            self.signal("finished")

    def cb_feedback(self, result):
        ActionState.progress = result.percent_complete
        print ActionState.progress
        if self.is_running():
            self.publish_state()

    def pause(self):
        try:
            self.pause_resume(1)
            return 'pause'
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return None

    def action_6(self, userdata):
        return self.pause()

    def set_status(self):
        if ActionState.progress >= 0:
            return "%d%% completed" % self.progress
        else:
            return "reaching initial position... "

    def new_instance(self, userdata):
        try:
            if userdata.resume:
                print userdata.resume
                return False
        except KeyError:
            return True

    def play(self, filename):
        ActionState.progress = 110
        ActionState.end = False
        ActionState.killing = False
        self.goal.msg.filename = filename
        self.goal.msg.loops = 1
        self.goal.msg.scale_vel = 100
        self.playback.send_goal(self.goal, self.cb_done, None, self.cb_feedback)

    def execute(self, userdata):
        if self.new_instance(userdata):
            self.play(userdata.filename)
        return ActionState.execute(self, userdata)

