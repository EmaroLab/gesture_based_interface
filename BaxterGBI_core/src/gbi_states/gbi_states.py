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

class FsmEvent():
	def __init__(self):
		self.trigger = threading.Event()
		self.event_id = None

def handle_config():
	return True

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
	def __init__(self, msg_type, callback, input_keys = [],output_keys=[]):
		smach.State.__init__(
			self,
			outcomes = ['invalid','success'],
			input_keys = input_keys,
			output_keys = output_keys)
		self._callback = callback
		print msg_type
		self._msg_type = msg_type
		print self._msg_type		
		self.subscribers=[]

	def execute(self, userdata):
		rospy.loginfo('Executing the state')

		for i in range(1,7):
			if not rospy.get_param("key_" + str(i) + "_topics"):
				return 'invalid' 
	
		for subscriber in self.subscribers:
			subscriber.unregister()
		self.subscribers = []

		for i in range(1,7):	
			for topic in rospy.get_param("key_" + str(i) + "_topics"):
				sub = rospy.Subscriber(	topic, 
												self._msg_type,
												self._callback, 
												{"topic": topic, "code": i}
				)
				self.subscribers.append(sub)

		return 'success'

class WaitConfigState(smach.State):
	def __init__(self, trigger_event, input_keys = [],output_keys=[]):
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
		while(True):
			self._trigger_event.trigger.clear()
			self._trigger_event.trigger.wait()
			print self._trigger_event.event_id
			if self._trigger_event.event_id == 'config':
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
			self._trigger_event.trigger.clear()
			self._trigger_event.trigger.wait()
			if self._trigger_event.event_id == 'user_detected':
				return 'user_dected'
			elif self._trigger_event.event_id == 'config':
				return 'reconf_requested'
