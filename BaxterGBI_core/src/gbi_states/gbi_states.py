import roslib
import rospy
from BaxterGBI_core_srvs.srv import *
import BaxterGBI_input_msgs.msg as bgi_io
import smach
import smach_ros
from std_msgs.msg import Empty
import threading
from collections import namedtuple

MenuContext=namedtuple('MenuContext', ['title','options','fixed_options','selction'])
ActionContext=namedtuple('ActionContext',['action','status'])


def publish_state(type,context=None):
	pass
	
config_available = False
event_id=None

class InitState(smach.State):
	def __init__(self,input_keys = [],output_keys=[]):
		smach.State.__init__(
			self,
			outcomes = ['config_available','config_missing'],
			input_keys = input_keys,
			output_keys = output_keys)

	def execute(self, userdata):
		global config_available
		return 'config_available' if config_available else 'config_missing'

class ConfigState(smach.State):
	def __init__(self,input_keys = [],output_keys=[]):
		smach.State.__init__(
			self,
			outcomes = ['invalid','success'],
			input_keys = input_keys,
			output_keys = output_keys)

	def execute(self, userdata):
		rospy.loginfo('Executing the state')
		return 'success' if handle_config() else 'invalid'

class WaitConfigState(smach.State):
	def __init__(self, trigger_event ,input_keys = [],output_keys=[]):
		smach.State.__init__(
			self,
			outcomes = ['config_available'],
			input_keys = input_keys,
			output_keys = output_keys)

		self._trigger_event = trigger_event
		self.type='config_wait'

	def execute(self, userdata):
		rospy.loginfo('Executing the state')
		publish_state(self.type)
		self._trigger_event.clear()
		self._trigger_event.wait()
		return 'config_available'


class WaitUserState(smach.State):
	def __init__(self, trigger_event ,input_keys = [],output_keys=[]):
		smach.State.__init__(
			self,
			outcomes = ['reconf_requested','user_detected'],
			input_keys = input_keys,
			output_keys = output_keys)

		self._trigger_event = trigger_event
		self.type='wait_user'

	def execute(self, userdata):
		rospy.loginfo('Executing the state')
		publish_state(self.type)
		while(True):
			self._trigger_event.clear()
			self._trigger_event.wait()
			if event_id == 'user_detected':
				return 'user_dected'
			elif event_id == 'config':
				return 'reconf_requested'
