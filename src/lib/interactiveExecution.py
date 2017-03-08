#!/usr/bin/python
#
# Interactive execution
#
import math
import os
import sys, code
import subprocess
import signal
import tempfile
import copy
import itertools
import numpy as np
import time
import strategy
import re
from collections import defaultdict

# logger for ltlmop
import logging
ltlmop_logger = logging.getLogger('ltlmop_logger')

class SLUGSInteractiveStrategy(strategy.Strategy):
    """
    An automaton object is a collection of state objects along with information about the
    current state of the automaton when being executed.
    """

    def __init__(self, slugsOptions):
        super(SLUGSInteractiveStrategy, self).__init__()

        self.states = strategy.StateCollection()
        self.slugsOptions = slugsOptions

    def _loadFromFile(self, filename):
        """
        Create an automaton by reading in a file produced by a synthesizer,
        such as JTLV or Slugs.

        Basically just a lot of regexes.
        """
        """
        if os.name == "nt":
            slugs_path = "\\".join(["etc", "slugs", "src", "slugs.exe"])
            filename = filename.replace('C:', '/cygdrive/c')  # .replace("\\", '/')
        else:
            slugs_path = os.path.join("etc", "slugs", "src", "slugs")
        ltlmop_logger.debug(slugs_path)
        """

        #command = " --interactiveStrategy --cooperativeGR1Strategy --sysInitRoboticsSemantics " # copy of old options in patching
        #ltlmop_logger.debug("slugs" + command + filename)
        ltlmop_logger.debug(self.slugsOptions)

        # Open Slugs
        self.slugsProcess = subprocess.Popen(' '.join(self.slugsOptions), shell=True, bufsize=1048000, stdin=subprocess.PIPE, stdout=subprocess.PIPE)

        # Get input APs
        self.slugsProcess.stdin.write("XPRINTINPUTS\n")
        self.slugsProcess.stdin.flush()
        self.slugsProcess.stdout.readline() # Skip the prompt
        lastLine = " "
        self.inputAPs = []
        while (lastLine!=""):
            lastLine = self.slugsProcess.stdout.readline().strip()
            if lastLine!="" and lastLine != ">":
                lastLine = re.sub(r'^sbit(\d+)$', r'regionCompleted_b\1', lastLine)
                self.inputAPs.append(lastLine)
        ltlmop_logger.debug( "inputAPs:" + str(self.inputAPs))

        # Get output APs
        self.slugsProcess.stdin.write("XPRINTOUTPUTS\n")
        self.slugsProcess.stdin.flush()
        self.slugsProcess.stdout.readline() # Skip the prompt
        lastLine = " "
        self.outputAPs = []
        while (lastLine!=""):
            lastLine = self.slugsProcess.stdout.readline().strip()
            if lastLine!="":
                lastLine = re.sub(r'^bit(\d+)$', r'region_b\1', lastLine)
                self.outputAPs.append(lastLine)
        ltlmop_logger.debug( "outputAPs:" + str(self.outputAPs))

    def getInitState(self):
        """
        This function requires no input and will return the initial state.
        """
        self.slugsProcess.stdin.write("XGETINIT\n")#XCOMPLETEINIT\n" + initInputsOutputs)#
        self.slugsProcess.stdin.flush()
        self.slugsProcess.stdout.readline() # Skip the prompt

        currentState = self.slugsProcess.stdout.readline().strip()
        ltlmop_logger.debug( "currentState:" + str(currentState))

        # create state with the current state prop assignments
        prop_assignments = {}
        for idx,element in enumerate(currentState.partition(",")[0]):
            value = True if element == '1' else False
            if idx > len(self.inputAPs)-1:
                prop_assignments[self.outputAPs[idx-len(self.inputAPs)]] = value
            else:
                prop_assignments[self.inputAPs[idx]] = value
        ltlmop_logger.debug("init_state:" + str([k for k, v in prop_assignments.iteritems() if v]))
        curStateObject = self.states.addNewState(prop_assignments = prop_assignments)

        # set current state id
        curStateObject.goal_id = currentState.partition(",")[2] # it's a string

        return (x for x in [curStateObject])

    def searchForStates(self, prop_assignments, state_list=None, goal_id=None):
        """ Returns an iterator for the subset of all known states (or a subset
            specified in `state_list`) that satisfy `prop_assignments`. """

        # first expand all domains
        prop_assignments = self.states.expandDomainsInPropAssignment(prop_assignments)

        ltlmop_logger.debug("prop_assignments for searching states:" + str([k for k, v in prop_assignments.iteritems() if v]))
        initInputsOutputs = ""
        for prop in self.inputAPs:
            if prop in prop_assignments.keys():
                initInputsOutputs += "1" if (prop_assignments[prop] is True) else "0"
            else:
                initInputsOutputs += "."
        for prop in self.outputAPs:
            if prop in prop_assignments.keys():
                initInputsOutputs += "1" if (prop_assignments[prop] is True) else "0"
            else:
                initInputsOutputs += "."

        ltlmop_logger.debug("initInputsOutputs:" + str(initInputsOutputs))
        self.slugsProcess.stdin.write("XCOMPLETEINIT\n" + initInputsOutputs)
        self.slugsProcess.stdin.flush()
        self.slugsProcess.stdout.readline() # Skip the prompt
        prompt = self.slugsProcess.stdout.readline().strip()

        # iterate until we actually get our state
        while re.search('[^aAgGsS01]',prompt):
            ltlmop_logger.debug("prompt:" + str(prompt))
            if 'FORCEDNONWINNING' in prompt:
                return []
            prompt = self.slugsProcess.stdout.readline().strip()
        currentState = prompt

        # in the form of AaGa
        # A: given true value,    a:given false value
        # G: possible true value, g:possible false value
        ltlmop_logger.debug( "currentState:" + str(currentState))

        # create state with the current state prop assignments
        prop_assignments = {}
        for idx,element in enumerate(currentState):
            value = True if element == 'A' or element == 'G' or element == '1' or element == 'S' else False
            if idx > len(self.inputAPs)-1:
                prop_assignments[self.outputAPs[idx-len(self.inputAPs)]] = value
            else:
                prop_assignments[self.inputAPs[idx]] = value
        ltlmop_logger.debug("init_state:" + str([k for k, v in prop_assignments.iteritems() if v]))
        curStateObject = self.states.addNewState(prop_assignments = prop_assignments)

        # set position in slugs
        self.slugsProcess.stdin.write("SETPOS\n" + currentState.replace("1","1\n").replace("0","0\n")\
                                                               .replace("A","1\n").replace("a","0\n")\
                                                               .replace("G","1\n").replace("g","0\n"))
        self.slugsProcess.stdin.flush()
        self.slugsProcess.stdout.readline() # only read Position:

        ltlmop_logger.debug("goal_id: {0}".format(goal_id))
        if goal_id is not None:
            ltlmop_logger.debug('rewriting goals... ')
            self.slugsProcess.stdin.write("XMAKEGOAL\n" + str(goal_id) + "\n")
            self.slugsProcess.stdin.flush()
            self.slugsProcess.stdout.readline() # only read Position:
            self.slugsProcess.stdout.readline() # only read Position:

        # get and set current state id
        self.slugsProcess.stdin.write("XGETCURRENTGOAL\n")
        self.slugsProcess.stdin.flush()
        currentGoal = self.slugsProcess.stdout.readline().partition(">")[2] #strip("> ")
        ltlmop_logger.debug("currentGoal:" + str(currentGoal))
        curStateObject.goal_id = currentGoal # it's a string

        return (x for x in [curStateObject])

    def findTransitionableStates(self, prop_assignments, from_state=None):
        """ Return a list of states that can be reached from `from_state`
            and satisfy `prop_assignments`.  If `from_state` is omitted,
            the strategy's current state will be used. """

        # first expand all domains
        prop_assignments = self.states.expandDomainsInPropAssignment(prop_assignments)

        # PICK THE NEXT VALUES HERE
        nextInput = ""
        for prop in self.inputAPs:
            nextInput += "1" if (prop_assignments[prop] is True) else "0"

        # Make the transition
        self.slugsProcess.stdin.write("XMAKETRANS\n"+nextInput)
        self.slugsProcess.stdin.flush()
        nextLine = self.slugsProcess.stdout.readline().strip() # Skip the prompt
        while nextLine.startswith(">"):
            nextLine = self.slugsProcess.stdout.readline().strip()
        if not nextLine.startswith("ERROR"):
            currentState = nextLine
        else:
            ltlmop_logger.debug("nextLine:" + str(nextLine))
            ltlmop_logger.debug("next inputs:" + str([k for k, v in prop_assignments.iteritems() if v]))
            ltlmop_logger.error("No next state!")
            return []

        #ltlmop_logger.debug("currentState:" + str(currentState))
        # create state with the current state prop assignments
        prop_assignments = {}
        for idx,element in enumerate(currentState.partition(",")[0]):
            value = True if element == '1' else False
            if idx > len(self.inputAPs) - 1:
                prop_assignments[self.outputAPs[idx-len(self.inputAPs)]] = value
            else:
                prop_assignments[self.inputAPs[idx]] = value

        curStateObject = strategy.State(self.states,prop_assignments)

        # set current state id
        curStateObject.goal_id = currentState.partition(",")[2] # it's a string

        return [curStateObject]


    def findTransitionableNextStates(self, from_state=None, current_goal_id=0):
        """ Return a list of states that can be reached from `from_state`
            and satisfy `prop_assignments`.  If `from_state` is omitted,
            the strategy's current state will be used. """

        self.slugsProcess.stdin.write("XGETPOSSIBLETRANS\n")
        self.slugsProcess.stdout.flush() # Skip the prompt
        self.slugsProcess.stdin.write(str(current_goal_id)+'\n')
        self.slugsProcess.stdin.flush()
        nextLine = self.slugsProcess.stdout.readline().strip()
        if "ERROR" in nextLine:
            ltlmop_logger.debug("nextLine:" + str(nextLine))
            ltlmop_logger.error("No next states!")
            return []

        nextPossibleStates = nextLine.split(':')[1].strip()
        #ltlmop_logger.debug("nextPossibleStates:" + str(nextPossibleStates))
        stateList = []

        # create state with the current state prop assignments
        for x in nextPossibleStates.split(',')[:-1]:
            prop_assignments = {}
            for idx, element in enumerate(x):
                value = True if element == '1' else False

                if idx > len(self.inputAPs) - 1:
                    prop_assignments[self.outputAPs[idx-len(self.inputAPs)]] = value
                else:
                    prop_assignments[self.inputAPs[idx]] = value

            curStateObject = strategy.State(self.states, prop_assignments)

            # set current state id
            curStateObject.goal_id = str(current_goal_id) # it's a string

            stateList.append(curStateObject)

        return stateList


if __name__ == '__main__':
    a = SLUGSInteractiveStrategy()
    a._loadFromFile('../examples/firefighting/firefighting.slugsin')
    a.searchForStates({})
    a.findTransitionableNextStates({}, 0)