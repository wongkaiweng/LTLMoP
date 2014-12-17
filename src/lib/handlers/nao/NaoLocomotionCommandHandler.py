#!/usr/bin/env python
"""
==================================================================
naoLocomotionCommand.py - Nao Locomotion Command Handler
==================================================================

Send forward, side, and angular velocity commands to the Nao.
"""

import threading

import lib.handlers.handlerTemplates as handlerTemplates

class NaoLocomotionCommandHandler(handlerTemplates.LocomotionCommandHandler):
    def __init__(self, executor, shared_data):
        """
        Locomotion Command handler for NAO robot.
        """

        self.naoInitHandler = shared_data['NAO_INIT_HANDLER']
        self.movProxy = self.naoInitHandler.createProxy('ALMotion')
        self.timer = None  # initialize timer for stopping action

    def sendCommand(self, cmd):
        """    Send movement command to the Nao
        Uses built in function in Aldebaran toolkit

        Expects [vel_x,vel_y,vel_ang,freq_step]"""

        # Extract data
        vx = cmd[0]
        vy = cmd[1]
        w = cmd[2]
        f = cmd[3]

        # kill timer if already exists
        if self.timer is None:
            pass
        elif self.timer.is_alive():
            self.timer.cancel()

        # Call robot movement
        self.movProxy.setWalkTargetVelocity(vx,vy,w,f)

        # start timer
        self.timer = threading.Timer(0.4, self.movProxy.setWalkTargetVelocity, args=[0, 0, 0, 0])
        self.timer.start()
