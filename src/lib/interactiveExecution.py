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
import logging
from collections import defaultdict

class SLUGSInteractiveStrategy(strategy.Strategy):
    """
    An automaton object is a collection of state objects along with information about the
    current state of the automaton when being executed.
    """

    def __init__(self):
        super(SLUGSInteractiveStrategy, self).__init__()

        self.states = strategy.StateCollection()

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
        logging.debug(slugs_path)
        """

        #command = " --interactiveStrategy --simpleRecovery --cooperativeGR1Strategy --sysInitRoboticsSemantics "
        command = " --interactiveStrategy --cooperativeGR1Strategy --sysInitRoboticsSemantics "
        logging.debug("slugs" + command + filename)

        # Open Slugs
        self.slugsProcess = subprocess.Popen("slugs" + command + filename, shell=True, bufsize=1048000, stdin=subprocess.PIPE, stdout=subprocess.PIPE)

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
        logging.debug( "inputAPs:" + str(self.inputAPs))

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
        logging.debug( "outputAPs:" + str(self.outputAPs))

    def searchForStates(self, prop_assignments, state_list=None):
        """ Returns an iterator for the subset of all known states (or a subset
            specified in `state_list`) that satisfy `prop_assignments`. """

        # HACK: currently perform no check
        self.slugsProcess.stdin.write("XGETINIT\n")#XCOMPLETEINIT\n" + initInputsOutputs)#
        self.slugsProcess.stdin.flush()
        self.slugsProcess.stdout.readline() # Skip the prompt
        currentState = self.slugsProcess.stdout.readline().strip()
        logging.debug( "currentState:" + str(currentState))

        # create state with the current state prop assignments
        prop_assignments = {}
        for idx,element in enumerate(currentState.partition(",")[0]):
            value = True if element == '1' else False
            if idx > len(self.inputAPs)-1:
                prop_assignments[self.outputAPs[idx-len(self.inputAPs)]] = value
            else:
                prop_assignments[self.inputAPs[idx]] = value
        logging.debug("init_state:" + str([k for k, v in prop_assignments.iteritems() if v]))
        curStateObject = self.states.addNewState(prop_assignments = prop_assignments)

        return (x for x in [curStateObject])

    def findTransitionableStates(self, prop_assignments, from_state=None):
        """ Return a list of states that can be reached from `from_state`
            and satisfy `prop_assignments`.  If `from_state` is omitted,
            the strategy's current state will be used. """

        # PICK THE NEXT VALUES HERE
        nextInput = ""
        for prop in self.inputAPs:
            nextInput += "1" if (prop_assignments[prop] is True) else "0"

        logging.debug("next inputs:" + str([k for k, v in prop_assignments.iteritems() if v]))
        # Make the transition
        self.slugsProcess.stdin.write("XMAKETRANS\n"+nextInput)
        self.slugsProcess.stdin.flush()
        self.slugsProcess.stdout.readline() # Skip the prompt
        nextLine = self.slugsProcess.stdout.readline().strip()
        if not nextLine.startswith("ERROR"):
            currentState = nextLine
        else:
            logging.debug("nextLine:" + str(nextLine))
            logging.error("No next state!")
            return []

        #logging.debug("currentState:" + str(currentState))
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
            logging.debug("nextLine:" + str(nextLine))
            logging.error("No next states!")
            return []

        nextPossibleStates = nextLine.split(':')[1].strip()
        #logging.debug("nextPossibleStates:" + str(nextPossibleStates))
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