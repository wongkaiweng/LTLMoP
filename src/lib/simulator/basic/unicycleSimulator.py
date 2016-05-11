#!/usr/bin/env python
"""
================================================================
unicycleSimulator.py -- A simple robot simulator provides pose by integrating given locomotion cmd
================================================================
"""
from numpy import array,sqrt,dot
from math import atan2,log10,ceil,sin,cos
import time, sys
import thread

class unicycleSimulator:
    def __init__(self, init_pose):
        """
        Initialization handler for pioneer ode simulated robot.

        init_pose is a 1-by-3 vector [x,y,orintation]
        """

        print "(Basic Unicycle Simulator) Initializing Basic Simulator with unicycle model..."
        self.pose = array(init_pose) # current pose
        self.curVel = array([0.0,0.0,0.0]) # current velocity
        self.time = 0.0 # used to calculate time elapsed
        self.inertia = 1 # scale from 0 to 1, the bigger the scale the smaller the "inertia" is
        self.setVel_called = False

        # Choose a timer func with maximum accuracy for given platform
        if sys.platform in ['win32', 'cygwin']:
            self.timer_func = time.clock
        else:
            self.timer_func = time.time

        print "(Basic Unicycle Simulator) Start Basic Simulator with unicycle model..."
        thread.start_new_thread(self.runSimulation, () )

    def setVel(self,cmd):
        """
        Set the velocity of the robot, update the pose by simply inegrate the velocity

        cmd is a 1-by-2 vector represents the velocity
        """

        # the orintation is kept the same (rad)
        # TODO: allows more robot models
        # update the velocity, assume the velocity takes times to change (to avoid local minimum)
        self.curVel = self.inertia*array(cmd)+(1-self.inertia)*self.curVel
        self.setVel_called = True

    def runSimulation(self):
        if self.time == 0.0:
            self.time = self.timer_func()
        while 1:
            if self.setVel_called:
                #print 'basicSim inputs vx:' + str(self.curVel[0]) + ' vy:' + str(self.curVel[1]) + ' w:' + str(self.curVel[2])
                time_span = (self.timer_func()-self.time)
                time_span = time_span*10**ceil(log10(0.03/time_span))
                vel = self.curVel[0]*array([cos(self.pose[2]), sin(self.pose[2])])  # x-component of the velocity
                vel += self.curVel[1]*array([-sin(self.pose[2]), cos(self.pose[2])])  # y-component of the velocity
                #print 'basic sim x/y velocity : '+str(vel)
                deltaCartesian = vel*time_span
                deltaOrientation = self.curVel[2]*time_span
                self.pose[0:2] = self.pose[0:2]+deltaCartesian
                self.pose[2] = self.pose[2]+deltaOrientation
                self.setVel_called=False
                #print 'basicSim outputs x,y:' + str(self.pose[0:2]) + ' theta:' + str(self.pose[2])
            else:
                self.pose[0:2] = self.pose[0:2]+array([0.0,0.0])*(self.timer_func()-self.time)
            self.time = self.timer_func()
            time.sleep(0.1)

    def getPose(self):
        """
        Returns the current pose of the robot
        """
        return self.pose
