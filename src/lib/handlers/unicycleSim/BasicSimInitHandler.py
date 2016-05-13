#!/usr/bin/env python
"""
================================================================
basicSimInit.py -- Basic Simulated Robot Initialization Handler
================================================================
"""
import lib.simulator.basic.unicycleSimulator as basicSimulator

import lib.handlers.handlerTemplates as handlerTemplates

import rospy

from lib.regions import Point

class BasicSimInitHandler(handlerTemplates.InitHandler):
    def __init__(self, executor, init_region, x=0., y=0., theta=0., absolute=False):
        """
        Initialization handler for basic simulated robot.

        init_region (region): The name of the region where the simulated robot starts
        x (float): the initial x-value in map coordinates (default=0.)
        y (float): the initial y-value in map coordinates (default=0.)
        theta (float): the initial theta-value in map coordinates (default=0.)
        absolute (bool): if True, treats the given coordinates as absolute; otherwise treats as an offset from the 'center' of the initial region (default=False)
        """

        rfi_original = executor.proj.loadRegionFile(decomposed=False)

        coordmap_map2lab = executor.hsub.coordmap_map2lab

        # Start in the center of the defined initial region
        init_region_obj = rfi_original.regions[rfi_original.indexOfRegionWithName(init_region)]
        center = init_region_obj.getCenter()
        if absolute:
            center.x = x
            center.y = y
        else:
            center.x = center.x + x
            center.y = center.y + y
        
        #initialize the simulator

        # uncomment the following to incorporate the map scaling for determining the initial pose in the unicycle model
        print "map to lab conversion of initial state : " + str(map(coordmap_map2lab, [Point(center.x, center.y)]))
        newPos = map(coordmap_map2lab, [Point(center.x, center.y)])[0]
        center.x = newPos[0]
        center.y = newPos[1]
        print [center.x, center.y]
        
        self.simulator =  basicSimulator.unicycleSimulator([center[0],center[1],theta])

        # rospy.init_node('LTLMoPHandlers')

    def getSharedData(self):
        # Return a dictionary of any objects that will need to be shared with
        # other handlers
        return {'UnicycleSimulator':self.simulator}


