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
import geometry_msgs.msg
import ast, numpy
import tf.transformations

import os, sys, time
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
        self.lastEgoPose = None
        self.lastEgoPoseTime = 0
        self.direction = [0,0]

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
            if agent not in self.agentPose.keys():
                self.agentPose[agent] = geometry_msgs.msg.Pose()
                rospy.Subscriber("/gazebo/model_states", gazebo_msgs.msg.ModelStates, self._callback, agent)

            # get transition data
            if not self.region_transition_dict:
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


    def checkIfAgentInSameLane(self, json_file_egents, initial=False):
        """
        This function checks if an environment agent is in same lane section
        json_file_egents (string): agents json file (default='fmr_autospec_eagents.json')
        """
        if initial:
            # open json file
           # open json file
            with open(json_file_egents,'r') as f:
                json_dict_list = ast.literal_eval(f.read())
            f.closed

            # get pose
            if "ego" not in self.agentPose.keys():
                self.agentPose["ego"] = geometry_msgs.msg.Pose()
                rospy.Subscriber("/gazebo/model_states", gazebo_msgs.msg.ModelStates, self._callback, "ego")

            for eagent_dict in json_dict_list:
                if eagent_dict["name"] not in self.agentPose.keys():
                    self.agentPose[eagent_dict["name"]] = geometry_msgs.msg.Pose()
                    rospy.Subscriber("/gazebo/model_states", gazebo_msgs.msg.ModelStates, self._callback, eagent_dict["name"])
            # get transition data
            if not self.region_transition_dict:
                f = open(self.executor.proj.rfi.filename.replace('.regions','_transition_dict.txt'), 'r')
                self.region_transition_dict = ast.literal_eval(f.read())
                f.close()
        else:
            try:
                # now check if agent is at the next region
                if self.executor.strategy:
                    currentRegionObj = self.executor.strategy.current_state.getPropValue('regionCompleted')
                    headingRegionObj = self.executor.strategy.current_state.getPropValue('region')
                    currentRegionPolygon = self._createRegionPolygon(currentRegionObj)
                    headingRegionPolygon = self._createRegionPolygon(headingRegionObj)

                    ego_pose = [self.agentPose["ego"].position.x, self.agentPose["ego"].position.y]

                    for agent in [x for x in self.agentPose.keys()if x != "ego"]:
                            # first transform pose
                            agent_pose = [self.agentPose[agent].position.x, self.agentPose[agent].position.y]

                            if currentRegionPolygon.isInside(agent_pose[0], agent_pose[1]):

                                # now check if agent is in front of me
                                ego_to_agent = numpy.array(agent_pose)-numpy.array(ego_pose)

                                # based on next region
                                #ego_heading = numpy.array(headingRegionPolygon.center(0))-numpy.array(ego_pose)
                                #ltlmop_logger.debug("ego_heading:" + str(ego_heading))
                                #ego_heading  = list(tf.transformations.euler_from_quaternion([self.agentPose["ego"].orientation.w,\
                                #    self.agentPose["ego"].orientation.x, self.agentPose["ego"].orientation.y, self.agentPose["ego"].orientation.z]))
                                #if numpy.dot(ego_to_agent,ego_heading[0:2]) > 0:

                                if numpy.dot(ego_to_agent,self.direction) > 0:
                                    ltlmop_logger.log(4,agent+' is in the same region in front of me.')
                                    return True
                                else:
                                    ltlmop_logger.log(2,agent+' is in the same region behind me.')

                    return False
                else:
                    return False
            except:
                return False

    def checkIfAgentAtIntsection(self, json_file_egents, initial=False):
        """
        This function checks if an environment agent is at an intersection
        (supposes to get the name from the trial file)
        json_file_egents (string): agents json file (default='fmr_autospec_eagents.json')
        """
        if initial:
            # open json file
            with open(json_file_egents,'r') as f:
                json_dict_list = ast.literal_eval(f.read())
            f.closed

            # get pose
            for eagent_dict in json_dict_list:
                if eagent_dict["name"] not in self.agentPose.keys():
                    self.agentPose[eagent_dict["name"]] = geometry_msgs.msg.Pose()
                    rospy.Subscriber("/gazebo/model_states", gazebo_msgs.msg.ModelStates, self._callback, eagent_dict["name"])

            # get transition data
            if not self.region_transition_dict:
                f = open(self.executor.proj.rfi.filename.replace('.regions','_transition_dict.txt'), 'r')
                self.region_transition_dict = ast.literal_eval(f.read())
                f.close()
        else:
             # now check if agent is at the next region
            try:
                if self.executor.strategy:
                    currentRegionObj = self.executor.strategy.current_state.getPropValue('regionCompleted')
                    for next_region_name in self.region_transition_dict[currentRegionObj.name]:
                        if 'full_intersect' in next_region_name:
                            # then obtain current heading region
                            headingRegionObj = self.executor.proj.rfi.regions[\
                                                    self.executor.proj.rfi.indexOfRegionWithName(next_region_name)]
                            headingRegionPolygon = self._createRegionPolygon(headingRegionObj)

                            for agent in [x for x in self.agentPose.keys() if x != "ego"]:
                                # first transform pose
                                #ltlmop_logger.debug(self.agentPose[agent])
                                simulation_pose = [self.agentPose[agent].position.x, \
                                                    self.agentPose[agent].position.y]

                                #ltlmop_logger.debug(agent+" in next region:" +\
                                #                     str(headingRegionPolygon.isInside(simulation_pose[0], simulation_pose[1])))
                                if headingRegionPolygon.isInside(simulation_pose[0], simulation_pose[1]):
                                    ltlmop_logger.log(4,agent+' is in the intersection front of me.')
                                    ltlmop_logger.debug("currentRegionObj.name:" + str(currentRegionObj.name))
                                    ltlmop_logger.debug("next_region_name:" + str(next_region_name))
                                    ltlmop_logger.debug("self.region_transition_dict[currentRegionObj.name]:" + str(self.region_transition_dict[currentRegionObj.name]))

                                    return True
                    return False
                else:
                    return False
            except:
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
        if msg is not None:
            egoIdx = msg.name.index(agent)
            self.agentPose[agent] = msg.pose[egoIdx]

            if self.lastEgoPose is None and agent == 'ego':
                self.lastEgoPose = self.agentPose['ego']
                self.lastEgoPoseTime = time.time()

            if agent == 'ego' and time.time() - self.lastEgoPoseTime < 1: #(in seconds)
                self.direction = [msg.pose[egoIdx].position.x - self.lastEgoPose.position.x,\
                                  msg.pose[egoIdx].position.y - self.lastEgoPose.position.y]
                self.lastEgoPose = msg.pose[egoIdx]
                self.lastEgoPoseTime = time.time()