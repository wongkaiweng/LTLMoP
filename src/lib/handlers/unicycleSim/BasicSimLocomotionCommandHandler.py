#!/usr/bin/env python
# -*- coding: cp1252 -*-
"""
================================================================================
basicSimLocomotionCommand.py - Basic Simulation Locomotion Command Handler
================================================================================
"""
import lib.handlers.handlerTemplates as handlerTemplates

class BasicSimLocomotionCommandHandler(handlerTemplates.LocomotionCommandHandler):
    def __init__(self, executor, shared_data,speed):
        """
        LocomotionCommand Handler for basic simulated robot.

        speed (float): The speed multiplier (default=1.0,min=6.0,max=15.0)
        """
        self.speed = speed
        try:
            self.simulator = shared_data['UnicycleSimulator']
        except KeyError:
            print "(Loco) ERROR: Basic Simulator doesn't seem to be initialized!"
            sys.exit(-1)

    def sendCommand(self, cmd):

        vx = self.speed*cmd[0]/100
        vy = self.speed*cmd[1]/100
        w = self.speed*cmd[2]

        print 'Locomotion Command handler inputs vx:' + str(vx) + 'vy:' + str(vy) + ' w:' + str(w)
        self.simulator.setVel([vx,vy,w])

