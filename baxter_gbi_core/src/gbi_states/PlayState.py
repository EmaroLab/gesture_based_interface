from ActionState import ActionState

class PlayState(ActionState):
    def __init__(self, trigger_event):
        input_keys = ['filename','resume']

        ActionState.__init__(self,
                             outcomes=['pause'],
                             trigger_event=trigger_event,
                             status='play',
                             output_keys=[],
                             input_keys=input_keys)
        

    def set_status(self):
        if self.progress >= 0:
            return "%d%% completed" % self.progress
        else:
            return "reaching initial position... "

    def execute(self,userdata):
        new_instance = True
        try:
            if (userdata.resume):
                new_instance = False
        except KeyError:
            pass
        if(new_instance):
            self.progress = 110
            self.goal.msg.filename = userdata.filename
            self.goal.msg.loops = 1;
            self.goal.msg.scale_vel = 100;
            self.playback.send_goal(self.goal, self.cb_done, None, self.feedback_cb)
        return ActionState.execute(self, userdata)