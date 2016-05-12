#!/usr/bin/env python
"""
====================================================
rosSensor.py - Sensor handler for the ROS interface
====================================================
"""

import roslib
import rospy
import Polygon
import gazebo_msgs.msg
import ast

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

# logger for ltlmop
import logging
ltlmop_logger = logging.getLogger('ltlmop_logger')

import lib.handlers.handlerTemplates as handlerTemplates

class RosSensorHandler(handlerTemplates.SensorHandler):
    def __init__(self, executor, shared_data):
        """
        ROS Sensor Handler
        """
        self.rosInitHandler = shared_data['ROS_INIT_HANDLER']
        self.executor = executor
        self.agentPose = {}
        self.region_transition_dict = {}

    ###################################
    ### Available sensor functions: ###
    ###################################

    def checkIfAgentAtNextRegion(self, agent, initial=False):
        """
        This function checks if an environment agent is at an intersection
        (supposes to get the name from the trial file)
        agent (string): name of the agent (default='agent0')
        """
        if initial:
            # get pose
            rospy.Subscriber("/gazebo/model_states", gazebo_msgs.msg.ModelStates, self._callback, agent)

            # get transition data
            f = open(self.executor.proj.rfi.filename.replace('.regions','_transition_dict.txt'), 'r')
            self.region_transition_dict = ast.literal_eval(f.read())
            f.close()
        else:
            # now check if agent is at the next region

            # first transform pose
            #ltlmop_logger.debug(self.agentPose[agent])
            simulation_pose = [self.agentPose[agent].position.x, \
                                self.agentPose[agent].position.y]

            # then obtain current heading region
            if self.executor.strategy:
                currentRegionObj = self.executor.strategy.current_state.getPropValue('regionCompleted')
                for next_region_name in self.region_transition_dict[currentRegionObj.name]:
                    headingRegionObj = self.executor.proj.rfi.regions[\
                                        self.executor.proj.rfi.indexOfRegionWithName(next_region_name)]
                    headingRegionPolygon = self._createRegionPolygon(headingRegionObj)

                    #ltlmop_logger.debug(agent+" in next region:" +\
                    #                     str(headingRegionPolygon.isInside(simulation_pose[0], simulation_pose[1])))
                    if headingRegionPolygon.isInside(simulation_pose[0], simulation_pose[1]):
                        ltlmop_logger.log(4,agent+' is in front of me.')
                        return True
                return False
            else:
                return False

    def _createRegionPolygon(self, region, hole=None):
        """
        This function takes in the region points and make it a Polygon.
        """
        if hole == None:
            pointArray = [x for x in region.getPoints()]
        else:
            pointArray = [x for x in region.getPoints(hole_id = hole)]
        pointArray = map(self.executor.hsub.coordmap_map2lab, pointArray)
        regionPoints = [(pt[0],pt[1]) for pt in pointArray]
        formedPolygon= Polygon.Polygon(regionPoints)
        return formedPolygon

    def _callback(self, msg, agent):
        egoIdx = msg.name.index(agent)
        self.agentPose[agent] = msg.pose[egoIdx]

