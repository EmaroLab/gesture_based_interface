#!/usr/bin/env python2.7

## @package parser
#. This package is the parser application of the
#  state machine

import yaml
import smach
import sys
import os

import gbi_states
from gbi_states import *

##  class State
#

class State:
   def __init__(self, name, class_type, remapping, parent):
      self.name = name
      self.class_type = class_type
      self.remapping = remapping
      self.parent = parent

##  class Machine
#   inherited from State

class Machine(State):
    ## the constructor
    #  @param name Machine's name
    #  @param parent Machine's parent
    #  @param outcomes Machine's outcomes
    #  @param initial_state Machine starting state
    #  @param input_keys machine input data embedded in userdata
    #  @param output_keys machine output data embedded in userdata
    #  @param class_type attribute of the Machine State
    #  @param remapping states IO remapping
   def __init__(self, name, parent, outcomes, initial_state, input_keys, output_keys, class_type, remapping):
      State.__init__(self, name, class_type, remapping, parent)
      self.outcomes = outcomes
      self.initial_state = initial_state
      self.input_keys = input_keys
      self.output_keys = output_keys
      self.states = []
      self.transitions = []
      self.children = []

    ## method add_state
    #  @param s state to add
   def add_state(self, s):
      self.states.append(s)

    ## method add_state
    #  @param t transition to add
   def add_transition(self, t):
      self.transitions.append(t)

    ## method transitions
    #
    #  method which return the set of whole transitions
   def transitions(self):
      return self.transitions

    ##  method transitions_from_node
    #   @param node node(which represent a state) of interest
   def transitions_from_node(self, node):
      ret_trans = []
      for t in self.transitions:
         if t.node_source == node:
            ret_trans.append(t)
      return ret_trans

    ##  method search_for_children
    #   @param orphans_machines
   def search_for_children(self, orphans_machines):
      for idx, machine in enumerate(orphans_machines):
         if machine.parent == self.name:
            self.children.append(machine)
            machine.search_for_children(orphans_machines)

## class Transition
#  

class Transition:
   ## the constructor
   #  @param node_source
   #  @param node_destination
   #  @param outcome
   def __init__(self, node_source, node_destination, outcome):
      self.node_source = node_source
      self.node_destination = node_destination
      self.outcome = outcome

## class InterFSMTransitions
#  
class InterFSMTransitions:
   ## the constructor
   def __init__(self):
      self.ift = []

    ##  method inter_fsm_transition
    #   @param transition
   def add_inter_fsm_transition(self, transition):
      self.ift.append(transition)

    ##  method transition_from_fsm
    #   @param fsm
   def transitions_from_fsm(self, fsm):
      ret_trans = []
      for t in self.ift:
         if t.node_source == fsm:
            ret_trans.append(t)
      return ret_trans

## class Parser
#  
class Parser:
   ## the constructor
   #  @param module
   #  @param event
   def __init__(self, module, event):
      self.classes = module
      self.event = event

    ##  method from_dict_to_list
    #   @param dict
   @staticmethod
   def from_dict_to_list(dict):
      return [value for key, value in dict.iteritems()] if dict else []

    ##  method from_dict_dict_to_list
    #   @param dict
   @staticmethod
   def from_dict_dict_to_list(dict):
      if not dict: return None
      ret_list = {}
      for key, value in dict.iteritems():
         ret_list.update({value['from']:value['to']})
      return ret_list

    ##  method verbose
    #   @param transitions
   @staticmethod
   def verbose(transitions):
      ret_list = {}
      for item in transitions:
         ret_list.update({item.outcome:item.node_destination})
      return ret_list

    ##  method parse
    # 
   def parse(self):
      try:
         open(os.path.dirname(os.path.realpath(__file__))+"/../fsm.yaml")
      except IOError as err:
         print ("fsm.yaml not found: " + err.strerror)
         sys.exit()
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

         head.search_for_children(plain_list_machines)

         self.sm, all_machines = self.init_fsm(head)
         all_machines.update({head.name:self.sm})
         return self.sm, all_machines

    ##  method init_fsm
    #   @param root
   def init_fsm(self, root):
      fsm = smach.StateMachine(outcomes=Parser.from_dict_to_list(root.outcomes),
                         input_keys=Parser.from_dict_to_list(root.input_keys))
      fsm.set_initial_state([str(root.initial_state)])

      fsm_children = {}
      with fsm:
         for state in root.states:
            #print(state.name)
            #print(state.class_type)
            smach.StateMachine.add(state.name, getattr(self.classes, state.class_type)(self.event),
                              Parser.verbose(root.transitions_from_node(state.name)),
                              Parser.from_dict_dict_to_list(state.remapping))
            #print(Parser.from_dict_dict_to_list(state.remapping))

         if root.children:
            for machine in root.children:
               fsm_child, fsm_nephews = self.init_fsm(machine)
               fsm_children.update({machine.name:fsm_child})
               fsm_children.update(fsm_nephews)
               smach.StateMachine.add(machine.name, fsm_child,
                  Parser.verbose(self.inter_fsm_transitions.transitions_from_fsm(machine.name)), Parser.from_dict_dict_to_list(machine.remapping))

      return fsm, fsm_children

