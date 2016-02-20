#!/usr/bin/env python
"""
==================================================================
BaxterLocomotionCommand.py - Baxter Locomotion Command Handler
==================================================================

Send forward, side, and angular velocity commands to the Baxter.
"""

import lib.handlers.handlerTemplates as handlerTemplates

class BaxterLocomotionCommandHandler(handlerTemplates.LocomotionCommandHandler):
    def __init__(self, executor, shared_data):
        """
        Locomotion Command handler for Baxter robot.
        """

        self.baxterInitHandler = shared_data['BAXTER_INIT_HANDLER']

    def sendCommand(self, cmd):
        """
        Currently the Baxter locomotion handler is a dummy handler,
        since baxter has no locomotion capabilities.
        """
        pass
