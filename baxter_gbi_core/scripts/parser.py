#!/usr/bin/env python2.7

## @package core
#. This package is the core application of the 
#  state machine

import yaml
import smach
import sys
import os

import gbi_states
from gbi_states import *

class State:
	def __init__(self, name, class_type, remapping, parent):
		self.name = name
		self.class_type = class_type
		self.remapping = remapping
		self.parent = parent

class Machine(State):
	def __init__(self, name, parent, outcomes, initial_state, input_keys, output_keys, class_type, remapping):
		State.__init__(self, name, class_type, remapping, parent)
		self.outcomes = outcomes
		self.initial_state = initial_state
		self.input_keys = input_keys
		self.output_keys = output_keys
		self.states = []
		self.transitions = []
		self.childs = []

	def add_state(self, s):
		self.states.append(s)

	def add_transition(self, t):
		self.transitions.append(t)

	def transitions(self):
		return self.transitions

	def transitions_from_node(self, node):
		ret_trans = []
		for t in self.transitions:
			if t.node_source == node:
				ret_trans.append(t)
		return ret_trans

	def search_for_childs(self, orphans_machines):
		for idx, machine in enumerate(orphans_machines):
			if machine.parent == self.name:
				self.childs.append(machine)
				machine.search_for_childs(orphans_machines)

class Transition:
	def __init__(self, node_source, node_destination, outcome):
		self.node_source = node_source
		self.node_destination = node_destination
		self.outcome = outcome

class InterFSMTransitions:
	def __init__(self):
		self.ift = []

	def add_inter_fsm_transition(self, transition):
		self.ift.append(transition)

	def transitions_from_fsm(self, fsm):
		ret_trans = []
		for t in self.ift:
			if t.node_source == fsm:
				ret_trans.append(t)
		return ret_trans

class Parser:
	def __init__(self, module, event):
		self.classes = module
		self.event = event

	@staticmethod
	def from_dict_to_list(dict):
		return [value for key, value in dict.iteritems()] if dict else []

	@staticmethod
	def from_dict_dict_to_list(dict):
		if not dict: return None
		ret_list = {}
		for key, value in dict.iteritems():
			ret_list.update({value['from']:value['to']})
		return ret_list

	@staticmethod
	def verbose(transitions):
		ret_list = {}
		for item in transitions:
			ret_list.update({item.outcome:item.node_destination})
		return ret_list

	def parse(self):
		with open(os.path.dirname(os.path.realpath(__file__))+"/../fsm.yaml") as stream:
			plain_list_machines = []

			self.inter_fsm_transitions = InterFSMTransitions()

			file = yaml.load(yaml.dump(yaml.load(stream)))
			machines = file['Machines']
			for key, machine in machines.iteritems():
				m = Machine(machine['name'], machine['parent'], machine['outcomes'], machine['initial_state'], machine['input_keys'], [], 'SmatchState', machine['remapping'])
				plain_list_machines.append(m)

				nodes = machine['nodes']
				for key, node in nodes.iteritems():
					n = State(node['name'], node['class'], node['remapping'], machine['name'])
					m.add_state(n)

				transitions = machine['transitions']
				for key, transition in transitions.iteritems():
					t = Transition(transition['from'], transition['to'], transition['event'])
					m.add_transition(t)

			inter_transitions = file['Inter-FSM-Transitions']
			for key, transition in inter_transitions.iteritems():
				t = Transition(transition['from'], transition['to'], transition['event'])
				self.inter_fsm_transitions.add_inter_fsm_transition(t)

			head = plain_list_machines[0]
			for idx, m in enumerate(plain_list_machines): 
				if m.parent == None:
					head = m
					del plain_list_machines[idx]
					break

			head.search_for_childs(plain_list_machines)

			self.sm, all_machines = self.init_fsm(head)
			all_machines.update({head.name:self.sm})
			return self.sm, all_machines

	def init_fsm(self, root):
		fsm = smach.StateMachine(outcomes=Parser.from_dict_to_list(root.outcomes),
								 input_keys=Parser.from_dict_to_list(root.input_keys))
		fsm.set_initial_state([str(root.initial_state)])

		fsm_childs = {}
		with fsm:
			for state in root.states:
				#print(state.name)
				#print(state.class_type)
				smach.StateMachine.add(state.name, getattr(self.classes, state.class_type)(self.event), 
										Parser.verbose(root.transitions_from_node(state.name)),
										Parser.from_dict_dict_to_list(state.remapping))
				#print(Parser.from_dict_dict_to_list(state.remapping))
			
			if root.childs:
				for machine in root.childs:
					fsm_child, fsm_nephews = self.init_fsm(machine)
					fsm_childs.update({machine.name:fsm_child})
					fsm_childs.update(fsm_nephews)
					smach.StateMachine.add(machine.name, fsm_child, 
						Parser.verbose(self.inter_fsm_transitions.transitions_from_fsm(machine.name)), Parser.from_dict_dict_to_list(machine.remapping))

		return fsm, fsm_childs

if __name__ == "__main__":
	event = FsmEvent()
	parser = Parser(gbi_states, event)
	#print(dir(gbi_states))
	parser.parse()

