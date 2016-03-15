#!/usr/bin/env python
"""
=================================================
naoInit.py - Nao Initialization Handler
=================================================

Initialize the proxies to access Nao modules
"""

import naoqi
from naoqi import ALProxy

import os, sys
# Climb the tree to find out where we are
p = os.path.abspath(__file__)
t = ""
while t != "src":
    (p, t) = os.path.split(p)
    if p == "":
        print "I have no idea where I am; this is ridiculous"
        sys.exit(1)

sys.path.append(os.path.join(p,"src","lib"))

import lib.handlers.handlerTemplates as handlerTemplates

class NaoInitHandler(handlerTemplates.InitHandler):
    def __init__(self, executor, ip='nao.local',port=9559):
        """
        Initialization handler for NAO robot.

        ip (string): The ip address of NAO (default="nao.local")
        port (int): The port of NAO (default=9559)
        """

        try:
            # Get connection settings from robot configuration file
            self.naoIP = ip
            self.naoPort = port


        except KeyError, ValueError:
            print "ERROR: Cannot find Nao connection settings ('NaoIP', 'NaoPort') in robot file."
            return None

    def createProxy(self, name):
        try:
            return ALProxy(name, self.naoIP, self.naoPort)
        except RuntimeError:
            print "ERROR: Cannot create %s proxy to Nao." % name
            print "ERROR: Make sure the Nao is turned on and connected to the network."
            return None

    def getSharedData(self):
        return {'NAO_INIT_HANDLER': self}

if __name__ == "__main__":
    import NaoActuatorHandler
    import NaoSensorHandler
    import time
    import sys

    #init = NaoInitHandler(None,"127.0.0.1", 36566)
    init = NaoInitHandler(None, "maeby.local", 9559)
    #init = NaoInitHandler(None, "maeby.local", 9559)
    act = NaoActuatorHandler.NaoActuatorHandler(None, init.getSharedData())
    sen = NaoSensorHandler.NaoSensorHandler(None, init.getSharedData())
    behaviorName = 'passitem-763a4f/behavior_1'
    #behaviorName = 'cook_good-4fa848/behavior_1'
    act.runBehavior(behaviorName, "", 0, initial=True)
    sen.isBehaviorCompleted(behaviorName, initial=True)
    print act.behaviorProxy.getInstalledBehaviors()
    stop = False


    # run pickup action
    while not stop:
        try:
            act.runBehavior(behaviorName, "", 1, initial=False)
            while not sen.isBehaviorCompleted(behaviorName):
                print behaviorName + 'is not completed.'
                time.sleep(0.5)
            print behaviorName + 'COMPLETED.'

            act.runBehavior(behaviorName, "", 0, initial=False)
            while sen.isBehaviorCompleted(behaviorName):
                print behaviorName + 'is not ended.'
                time.sleep(0.5)
            print behaviorName + 'ENDED.'

        except KeyboardInterrupt:
            stop = True
            sys.exit()

