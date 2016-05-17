#!/usr/bin/env python
"""
=================================================
rosSim.py - ROS/Gazebo Initialization Handler
=================================================
"""
import math
import sys, subprocess, os, time, os, shutil, rospy
import re, Polygon, Polygon.IO

# Climb the tree to find out where we are
p = os.path.abspath(__file__)
t = ""
while t != "src":
    (p, t) = os.path.split(p)
    if p == "":
        print "I have no idea where I am; this is ridiculous"
        sys.exit(1)

sys.path.append(os.path.join(p,"src","lib"))

import lib.regions as regions
# logger for ltlmop
import logging
ltlmop_logger = logging.getLogger('ltlmop_logger')

import gazebo_msgs.msg, gazebo_msgs.srv
import numpy

import lib.handlers.handlerTemplates as handlerTemplates

class RosInitHandler(handlerTemplates.InitHandler):
    def __init__(self, executor, init_region, robotPixelWidth, robotPhysicalWidth):
        """
        Initialization handler for ROS with dublins car

        init_region (region): The name of the region where the simulated robot starts
        robotPixelWidth (int): The width of the robot in pixels in ltlmop (default=200)
        robotPhysicalWidth (float): The physical width of the robot in meters (default=.2)
        """
        self.executor = executor
        self.robotPhysicalWidth = robotPhysicalWidth
        self.robotPixelWidth = robotPixelWidth

        #The following is a global node for LTLMoP
        rospy.init_node('LTLMoPHandlers')

        # set robot start location
        ltlmop_logger.debug("init_region:" + str(init_region))
        if init_region:
            self.centerTheRobot(init_region)

    def getSharedData(self):
        # TODO: Return a dictionary of any objects that will need to be shared with other handlers
        return {'ROS_INIT_HANDLER': self}

    def createRegionPolygon(self,region,hole = None):
        """
        This function takes in the region points and make it a Polygon.
        """
        if hole == None:
            pointArray = [x for x in region.getPoints()]
        else:
            pointArray = [x for x in region.getPoints(hole_id = hole)]
        pointArray = map(self.coordmap_map2lab, pointArray)
        regionPoints = [(pt[0],pt[1]) for pt in pointArray]
        formedPolygon= Polygon.Polygon(regionPoints)
        return formedPolygon

    def centerTheRobot(self, init_region):
        # Start in the center of the defined initial region

        rfi_original = self.executor.proj.loadRegionFile(decomposed=False)

        # Start in the center of the defined initial region
        init_region_obj = rfi_original.regions[rfi_original.indexOfRegionWithName(init_region)]
        center = init_region_obj.getCenter()

        # Load the map calibration data and the region file data to feed to the simulator
        coordmap_map2lab,coordmap_lab2map = self.executor.hsub.getMainRobot().getCoordMaps()
        map2lab = list(coordmap_map2lab(numpy.array(center)))

        #ltlmop_logger.log(4,"Initial region name: "+ init_region+ " I think I am here: "+\
        #                    str(map2lab) + " and center is: " + str(center))


        #pub = rospy.Publisher('/gazebo/set_model_state', gazebo_msgs.msg.ModelState, queue_size=10)
        currentState = gazebo_msgs.msg.ModelState()
        currentState.model_name = 'ego'
        currentState.pose.position.x = map2lab[0]
        currentState.pose.position.y = map2lab[1]
        currentState.reference_frame = 'world'

        service_ego = rospy.ServiceProxy('/gazebo/set_model_state', gazebo_msgs.srv.SetModelState)
        output = service_ego(currentState)

