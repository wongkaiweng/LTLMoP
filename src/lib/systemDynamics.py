#!/usr/bin/env python

import numpy as np

class SystemDynamics():
    """
    Specifies dynamics-depedent methods for an arbitrary system 
    """

    def __init__(self, dimConfigSpace, dimStateSpace, dimInputSpace):
        
        self.numConfig = dimConfigSpace
        self.numState = dimStateSpace
        self.numInput = dimInputSpace

        self.uLim = np.array([np.inf] * dimInputSpace)

        self.isCyclic = []

    def state2SEconfig(self, x):
        """
        Transforms a state vector to an SE(n) configuration (x,y translations plus rotation)
        """

        y = x[:self.numConfig+1]

        return y

    def SEconfig2state(self, y):
        """
        Transforms SE(n) configuration vector to a state vector
        """

        x = np.array([0]*self.numState)
        x[:self.numConfig+1] = y

        return x

class Holonomic(SystemDynamics):
    """
    Specifies dynamics-depedent methods for a holonomic-drive robot
    """
    def __init__(self):
        SystemDynamics.__init__(self,2,2,2)

        self.isCyclic = [0, 0]

    def state2SEconfig(self, x):
        """
        Transforms a state vector to an SE(n) configuration (x,y translations plus rotation)
        """
        
        y = SystemDynamics.state2SEconfig(self,x)
        y[2] = 0

        return y

    def SEconfig2state(self, y):
        """
        Transforms SE(n) configuration vector to a state vector
        """

        x = SystemDynamics.SEconfig2state(self, y)
        
        return x

    def command2robotInput(self, x, u):

        z = state2SEconfig(x)

        u_local = u*[
            [np.cos(-z[2]), -np.sin(-z[2])],
            [np.sin(-z[2]), np.cos(-z[2])]]
        vx = u_local[0]
        vy = u_local[1]
        w = u[2]

        return vx, vy, w

class Unicycle(SystemDynamics):
    """
    Specifies dynamics-depedent methods for a unicycle
    """
    def __init__(self):
        SystemDynamics.__init__(self,2,3,2)

        # The following sets the +/- limits for the input command
        self.uLim = np.array([np.inf, 10])

        self.isCyclic = [0, 0, 1]

    def state2SEconfig(self, x):
        """
        Transforms a state vector to an SE(n) configuration (x,y translations plus rotation)
        """

        y = SystemDynamics.state2SEconfig(self,x)

        return y

    def SEconfig2state(self, y):
        """
        Transforms SE(n) configuration vector to a state vector
        """

        x = SystemDynamics.SEconfig2state(self, y)

        return x

    def command2robotInput(self, x, u):

        vx = 0.08 #u[0]
        vy = 0.
        print 'Unicycle commands:'
        print u, u[0], self.uLim[0]
        w = max(min([u[0]], self.uLim[0]), -self.uLim[0])[0]

        return vx, vy, w

if __name__ == "__main__":
    # cmd-line argument checking, etc.
    SystemDynamics(0,0,0)
