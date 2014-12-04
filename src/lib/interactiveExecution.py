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
import socket
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

        self.inputAPs = self.getInputs()
        self.outputAPs = self.getOutputs()
        logging.debug("inputAPs:" + str(self.inputAPs))
        logging.debug("outputAPs:" + str(self.outputAPs))
#===============================================================================
#         if os.name == "nt":
#             slugs_path = "\\".join(["etc", "slugs", "src", "slugs.exe"])
#             filename = filename.replace('C:', '/cygdrive/c')  # .replace("\\", '/')
#         else:
#             slugs_path = os.path.join("etc", "slugs", "src", "slugs")
#         logging.debug(slugs_path)
#         logging.debug("filename:" + str(filename))
#         logging.debug(slugs_path + " --interactiveStrategy " + filename)
#
#         # Open Slugs
#         self.slugsProcess = subprocess.Popen(slugs_path + " --interactiveStrategy " + filename, shell=True, bufsize=1048000, stdin=subprocess.PIPE, stdout=subprocess.PIPE)
#
#         # Get input APs
#         self.slugsProcess.stdin.write("XPRINTINPUTS\n")
#         self.slugsProcess.stdin.flush()
#         self.slugsProcess.stdout.readline()  # Skip the prompt
#         lastLine = " "
#         self.inputAPs = []
#         while (lastLine != ""):
#             lastLine = self.slugsProcess.stdout.readline().strip()
#             if lastLine != "":
#                 self.inputAPs.append(lastLine)
#         logging.debug("inputAPs:" + str(self.inputAPs))
#
#         # Get output APs
#         self.slugsProcess.stdin.write("XPRINTOUTPUTS\n")
#         self.slugsProcess.stdin.flush()
#         self.slugsProcess.stdout.readline()  # Skip the prompt
#         lastLine = " "
#         self.outputAPs = []
#         while (lastLine != ""):
#             lastLine = self.slugsProcess.stdout.readline().strip()
#             if lastLine != "":
#                 lastLine = re.sub(r'^bit(\d+)$', r'region_b\1', lastLine)
#                 self.outputAPs.append(lastLine)
#         logging.debug("outputAPs:" + str(self.outputAPs))
#===============================================================================

    def searchForStates(self, prop_assignments, state_list=None):
        """ Returns an iterator for the subset of all known states (or a subset
            specified in `state_list`) that satisfy `prop_assignments`. """

        #=======================================================================
        # # HACK: currently perform no check
        # self.slugsProcess.stdin.write("XGETINIT\n")  # XCOMPLETEINIT\n" + initInputsOutputs)#
        # self.slugsProcess.stdin.flush()
        # self.slugsProcess.stdout.readline()  # Skip the prompt
        # currentState = self.slugsProcess.stdout.readline().strip()
        #=======================================================================
        currentState = self.getState()
        logging.debug("currentState:" + str(currentState))

        # create state with the current state prop assignments
        prop_assignments = {}
        for idx, element in enumerate(currentState.partition(",")[0]):
            value = True if element == '1' else False
            if idx > len(self.inputAPs) - 1:
                prop_assignments[self.outputAPs[idx - len(self.inputAPs)]] = value
            else:
                prop_assignments[self.inputAPs[idx]] = value
        logging.debug(prop_assignments)
        curStateObject = self.states.addNewState(prop_assignments=prop_assignments)

        return (x for x in [curStateObject])

    def findTransitionableStates(self, prop_assignments, from_state=None):
        """ Return a list of states that can be reached from `from_state`
            and satisfy `prop_assignments`.  If `from_state` is omitted,
            the strategy's current state will be used. """

        # PICK THE NEXT VALUES HERE
        nextInput = ""
        for prop in self.inputAPs:
            nextInput += "1" if (prop_assignments[prop] is True) else "0"
        logging.debug("nextInput:" + str(nextInput))
        # Make the transition
        #=======================================================================
        # self.slugsProcess.stdin.write("XMAKETRANS\n" + nextInput)
        # self.slugsProcess.stdin.flush()
        # self.slugsProcess.stdout.readline()  # Skip the prompt
        # nextLine = self.slugsProcess.stdout.readline().strip()
        #=======================================================================
        nextLine = self.getTrans(nextInput)
        if not nextLine.startswith("ERROR"):
            currentState = nextLine
        logging.debug("currentState:" + str(currentState))
        # create state with the current state prop assignments
        prop_assignments = {}
        for idx, element in enumerate(currentState.partition(",")[0]):
            value = True if element == '1' else False

            if idx > len(self.inputAPs) - 1:
                prop_assignments[self.outputAPs[idx - len(self.inputAPs)]] = value
            else:
                prop_assignments[self.inputAPs[idx]] = value

        curStateObject = strategy.State(self.states, prop_assignments)

        return [curStateObject]

    def talkToServer(self, msg):
        """
        Talk to slug server
        
        msg options: inputs, outputs, state, trans
        """
        HOST, PORT = "localhost", 9999
        data = msg

        # Create a socket (SOCK_STREAM means a TCP socket)
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        received = ""
        try:
            # Connect to server and send data
            sock.connect((HOST, PORT))
            sock.sendall(data + "\n")

            # Receive data from the server and shut down
            received = sock.recv(1024)
            logging.debug("Got message {}".format(received))
        finally:
            sock.close()

        return received

    def getInputs(self):
        """
        Get inputs props from slugs
        return a list of inputs prop names
        """
        data = self.talkToServer("inputs")
        return data.split(",")

    def getOutputs(self):
        """
        Get outputs props from slugs
        return a list of outputs prop names
        """
        data = self.talkToServer("outputs")
        return data.split(",")

    def getState(self):
        """
        Get a state from slugs
        return a string
        """
        data = self.talkToServer("state")
        return data

    def getTrans(self, nextInput):
        """
        Get a transition state from slugs
        return a string
        """
        data = self.talkToServer("trans:{}".format(nextInput))
        return data
