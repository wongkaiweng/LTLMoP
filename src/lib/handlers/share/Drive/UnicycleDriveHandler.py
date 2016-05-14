#!/usr/bin/env python
"""
=======================================================
UnicycleDrive.py - Unicycle Drive Handler
=======================================================

"""

import lib.handlers.handlerTemplates as handlerTemplates

class UnicycleDriveHandler(handlerTemplates.DriveHandler):
    def __init__(self, executor, shared_data,multiplier,maxspeed):
        """
        Passes velocity requests directly through.
        Used for ideal holonomic point robots.

        multiplier (float): Scale the velocity from motionControlHandler (default=1.0,min=1.0,max=50.0)
        maxspeed (float): Max speed allowed (default=999.0)
        """
        try:
            self.loco = executor.hsub.getHandlerInstanceByType(handlerTemplates.LocomotionCommandHandler)
            self.coordmap = executor.hsub.coordmap_lab2map
        except NameError:
            print "(DRIVE) Locomotion Command Handler not found."
            exit(-1)

        self.mul = multiplier
        self.max = maxspeed

    def setVelocity(self, vx, vy, w, theta=0):
        vx = max(min(vx*self.mul,self.max),-self.max)
        vy = max(min(vy*self.mul,self.max),-self.max)

        #print "VEL:%f,%f" % tuple(self.coordmap([x, y]))
        # print 'Drive handler inputs x:' + str(x) + ' y:' + str(y)
        self.loco.sendCommand([vx,vy,w])

