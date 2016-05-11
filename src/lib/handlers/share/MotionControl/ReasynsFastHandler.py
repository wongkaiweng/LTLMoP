#!/usr/bin/env python
"""
=======================================================
ReasynsFastHandler.py 

This version of the handler takes in a set of funnels that are templates (primatives) that are rotated and translated.
=======================================================

"""

from numpy import *
from __is_inside import *
import time, math, sys
import logging
import __ReasynsHelperFast as Reasyns
from collections import OrderedDict
from scipy.linalg import norm
import Polygon, Polygon.IO
import Polygon.Utils as PolyUtils
import Polygon.Shapes as PolyShapes
import project
import os
import time

import lib.handlers.handlerTemplates as handlerTemplates
from lib.regions import Point

class ReasynsFastHandler(handlerTemplates.MotionControlHandler):
    def __init__(self, executor, shared_data, scalingPixelsToMeters, fname):
        """
        Vector motion planning controller

        scalingPixelsToMeters (float): Scaling factor between RegionEditor map and the Matlab map
        fname (string): Name of the .mat file containing the stored atomic controllers
        """

        # filename                = '/home/jon/Dropbox/Repos/LTLMoP/src/lib/handlers/share/MotionControl/resultsPython'
        filename                = os.getcwd()+'/examples/fmrChallenge/reasyns_controllers/'+fname
        if not os.path.exists(filename):
            logging.exception('Cannot open the specified reasyns controller file.')

        self.numRobots          = []    # number of robots: number of agents in the specification, controlled by the local planner
        
        self.scalingPixelsToMeters = scalingPixelsToMeters
        self.updateWithPose     = {}
        self.forceUpdate        = False
        self.initial            = True
        self.previous_next_reg  = {}
        self.previous_curr_reg  = {}
        self.pose               = {}
        self.current_regVertices= {}
        self.next_regVertices   = {}
        self.system_print       = False       # for debugging. print on GUI ( a bunch of stuffs)
        self.Velocity           = None
        self.currentRegionPoly  = None
        self.nextRegionPoly     = None
        self.radius             = self.scalingPixelsToMeters*0.15
        
        self.maxIntersects      = 3
        self.roadSegmentLength  = 450/30
        self.vertsTemplate      = {}
        self.vertsTemplate[0]   = mat([[60/30,60/30,1],[0,60/30,1],[0,240/30,1],[60/30,240/30,1]])  #vertices for matching a transformation matrix to a pre-generated template
        self.invTransformationMatrix = linalg.inv(mat([[1,0,0],[0,1,0],[0,0,1]]))   # transformation matrix for the template
        self.prevInvTransformationMatrix = linalg.inv(mat([[1,0,0],[0,1,0],[0,0,1]]))   # transformation matrix for the template
        self.intersectionSuccessor = []
        self.templateIndex         = 0
          #TODO: get this directly from the .regions file used to generate the primitives

        self.pose = OrderedDict()
        
        # get the list of robots
        self.robotList = [robot.name for robot in executor.hsub.executing_config.robots]
        if not self.numRobots == len(self.robotList):
            self.numRobots = len(self.robotList)
            # self.numRobots = 3
            print "WARNING: changing the number of robots to "+str(self.numRobots)+" to match the config file"
        for robot_name in self.robotList:
            self.updateWithPose[robot_name] = False
        
        self.drive_handler = {}
        self.pose_handler  = {}
        self.executor      = executor
        # Get references to handlers we'll need to communicate with
        for robot_name in self.robotList: # x must be a string
            self.drive_handler[robot_name] = executor.hsub.getHandlerInstanceByType(handlerTemplates.DriveHandler, robot_name)
            self.drive_handler[robot_name].loco = executor.hsub.getHandlerInstanceByType(handlerTemplates.LocomotionCommandHandler, robot_name)
            self.pose_handler[robot_name] = executor.hsub.getHandlerInstanceByType(handlerTemplates.PoseHandler, robot_name)

            self.previous_curr_reg[robot_name] = 12
            self.previous_next_reg[robot_name] = 10

        # Get information about regions
        self.rfi = executor.proj.rfi
        self.coordmap_map2lab = executor.hsub.coordmap_map2lab
        self.coordmap_lab2map = executor.hsub.coordmap_lab2map 
        self.last_warning = 0

        self.map = {}
        for region in self.rfi.regions:
            self.map[region.name] = self.createRegionPolygon(region)

        # Initialize the data structures
        sysObj, acLastData, data, aut, ac_trans, ac_inward, cyclicTrinaryVector = Reasyns.initializeController(filename)

        self.sysObj = sysObj
        self.acLastData = acLastData
        self.data = data
        self.aut = aut
        self.ac_trans = ac_trans
        self.ac_inward = ac_inward
        self.cyclicTrinaryVector = cyclicTrinaryVector
        self.currRegNbr = 4
        self.nextRegNbr = 0


    def gotoRegion(self, current_regIndices, next_regIndices, last=False):
        """
        If ``last`` is True, we will move to the center of the destination region.

        Returns ``True`` if we've reached the destination region.

        current_regAllIndices: dictionary of region indices
        next_regAllIndices: dictionary of region indices
        """

        departed            = {}
        arrived             = {}
        doUpdate            = {}


        logging.debug(self.executor.strategy.states.index(self.executor.strategy.current_state))
        logging.debug(self.executor.strategy.states.index(self.executor.next_state))

        logging.debug("current_regIndices:" + str(current_regIndices))
        logging.debug("next_regIndices: " + str(next_regIndices))
        print "current_regIndices:" + str(current_regIndices)
        print "next_regIndices: " + str(next_regIndices)

        for robot_name in self.robotList:

            current_reg = current_regIndices
            next_reg = next_regIndices

            self.pose.update([(robot_name,self.pose_handler[robot_name].getPose())])
            # print "pose: "+str(self.pose[robot_name])

            doUpdate[robot_name] = False 
            if not self.previous_curr_reg[robot_name] == current_reg or not self.previous_next_reg[robot_name] == next_reg:
                # Find our current configuration
                # self.pose.update([(robot_name,self.pose_handler[robot_name].getPose())])
                # print "pose: "+str(self.pose[robot_name])

                # Check if Vicon has cut out
                # TODO: this should probably go in posehandler?
                if math.isnan(self.pose[robot_name][2]):
                    print "WARNING: No Vicon data! Pausing."
                    self.drive_handler[robot_name].setVelocity(0, 0, 0)  # So let's stop
                    time.sleep(1)
                    #return False not leaving yet until all robots are checked

                # if self.system_print == True:
                print "Next Region for " +str(robot_name)+" is: "+str(self.rfi.regions[next_reg].name)
                print "Current Region for " +str(robot_name)+" is: "+str(self.rfi.regions[current_reg].name)
                logging.debug("Next Region is " + str(self.rfi.regions[next_reg].name))
                logging.debug("Current Region is " + str(self.rfi.regions[current_reg].name))

                self.currentRegionPoly = self.map[self.rfi.regions[current_reg].name]
                regionPolyOld = Polygon.Polygon(self.currentRegionPoly)

                # NOTE: Information about region geometry can be found in self.rfi.regions:
                vertices = mat(map(self.coordmap_map2lab, [x for x in self.rfi.regions[current_reg].getPoints()])).T
                # have to do this twice??
                tmp = [Point(y[0],y[1]) for y in map(self.coordmap_map2lab, [x for x in self.rfi.regions[current_reg].getPoints()])]
                #vertices = mat(map(self.coordmap_map2lab, tmp)).T

                self.current_regVertices[robot_name] = vertices

                vertices = mat(map(self.coordmap_map2lab, [x for x in self.rfi.regions[next_reg].getPoints()])).T
                # have to do this twice??
                tmp = [Point(y[0],y[1]) for y in map(self.coordmap_map2lab, [x for x in self.rfi.regions[next_reg].getPoints()])]
                #print [x for x in self.rfi.regions[next_reg].getPoints()]
                #print tmp
                #vertices = mat(map(self.coordmap_map2lab, tmp)).T
                #print vertices

                self.next_regVertices[robot_name] = vertices

                print "vertices: "+str(self.next_regVertices[robot_name])

                if not current_reg == next_reg:
                    doUpdate[robot_name] = True

                    self.intersectionSuccessor = []

                    # If we've just entered a 'lane' from an 'intersection' (rising edge), then compute a new transformation matrix
                    print self.rfi.regions[current_reg].name
                    print self.rfi.regions[next_reg].name
                    if 'intersect' in self.rfi.regions[current_reg].name and 'lane' in self.rfi.regions[next_reg].name:
                        templateIndex, transformationMatrix, thIndex = self.getTransformationMatrixAndTemplate(mat(vertices)) 
                        self.templateIndex = templateIndex
                        self.prevInvTransformationMatrix = linalg.inv(self.transformationMatrix)
                        self.invTransformationMatrix = linalg.inv(transformationMatrix)

                    elif 'lane' in self.rfi.regions[self.previous_next_reg[robot_name]].name and 'intersect' in self.rfi.regions[next_reg].name:
                        print "vertices: " + str(vertices)
                        # templateIndex, transformationMatrix, thIndex = self.getTransformationMatrixAndTemplate(mat([[60,60],[0,60],[0,240],[60,240]]))
                        templateIndex, transformationMatrix, thIndex = self.getTransformationMatrixAndTemplate(mat(self.next_regVertices[robot_name]))
                        print "found template: " + str(transformationMatrix)
                        self.intersectionSuccessor = thIndex-1


                # store the previous region designations
                self.previous_next_reg[robot_name] = next_reg
                self.previous_curr_reg[robot_name] = current_reg

        # Run algorithm to find a velocity vector (global frame) to take the robot to the next region
        vx, vy, w, acLastData, data, currRegNbr, nextRegNbr = Reasyns.executeController(self.sysObj, self.pose, self.rfi.regions, \
            current_regIndices, next_regIndices, self.coordmap_lab2map, self.scalingPixelsToMeters, doUpdate, self.acLastData, self.data, \
            self.aut, self.ac_trans, self.ac_inward, self.cyclicTrinaryVector,self.currRegNbr,self.nextRegNbr, \
            self.templateIndex, self.prevInvTransformationMatrix, self.invTransformationMatrix, self.intersectionSuccessor)

        self.acLastData = acLastData
        self.data = data
        self.currRegNbr = currRegNbr
        self.nextRegNbr = nextRegNbr

        for idx, robot_name in enumerate(self.robotList):
            # current_reg = current_regIndices[robot_name]
            current_reg = current_regIndices

            logging.debug(robot_name + '-vx:' + str(vx[idx]) + '-vy:' + str(vy[idx]) + ' w:' + str(w[idx]))
            self.drive_handler[robot_name].setVelocity(vx[idx], vy[idx], w[idx], self.pose[robot_name][2])

            #logging.debug("pose:" + str(pose))
            # departed[robot_name] = not is_inside([self.pose[robot_name][0], self.pose[robot_name][1]], self.current_regVertices[robot_name])
            # # Figure out whether we've reached the destination region
            # # arrived[robot_name] = is_inside([self.pose[robot_name][0], self.pose[robot_name][1]], self.next_regVertices[robot_name])
            # arrived[robot_name] = False
            # for ireg in range(len(self.rfi.regions)):
            #     if ireg != current_reg:
            #         vertices = mat(map(self.coordmap_map2lab, [x for x in self.rfi.regions[ireg].getPoints()])).T
            #         arrived[robot_name] |= is_inside([self.pose[robot_name][0], self.pose[robot_name][1]], vertices)

            # if departed[robot_name] and (not arrived[robot_name]) and (time.time()-self.last_warning) > 0.5:
            #     #print "WARNING: Left current region but not in expected destination region"
            #     # Figure out what region we think we stumbled into
            #     for r in self.rfi.regions:
            #         pointArray = [self.coordmap_map2lab(x) for x in r.getPoints()]
            #         vertices = mat(pointArray).T
            #         if is_inside([self.pose[robot_name][0], self.pose[robot_name][1]], vertices):
            #             logging.info("I think I'm in " + r.name)
            #             break
            #     self.last_warning = time.time()

        # if norm([v[idx] for idx in range(len(v))]) < 0.01:
        #     self.forceUpdate = True

        self.initial = False
        logging.debug("arrived:" + str(arrived))
        return (True in arrived.values()) #arrived[self.executor.hsub.getMainRobot().name]

    def createRegionPolygon(self,region,hole = None):
        if hole == None:
            pointArray = [x for x in region.getPoints()]
        else:
            pointArray = [x for x in region.getPoints(hole_id = hole)]
        pointArray = map(self.coordmap_map2lab, pointArray)
        regionPoints = [(float(1)/self.scalingPixelsToMeters*pt[0],float(1)/self.scalingPixelsToMeters*pt[1]) for pt in pointArray]
        formedPolygon= Polygon.Polygon(regionPoints)
        return formedPolygon

    def getRegionVertices(self,region):
        if type(region).__name__ == 'Region':
            pointArray = [x for x in region.getPoints()]
        else:
            pointArray = region
        pointArray = map(self.coordmap_map2lab, pointArray)
        regionPoints = [(float(1)/self.scalingPixelsToMeters*pt[0],float(1)/self.scalingPixelsToMeters*pt[1]) for pt in pointArray]
        return regionPoints

    def getTransformationMatrixAndTemplate(self,nextRegionVertices):
        # search for both the funnels template and an appropriate transformation matrix to apply for the road segment we've just entered
        for xIndex in range(self.maxIntersects):
            for yIndex in range(self.maxIntersects):
                for thIndex in range(4):
                    theta = (thIndex+1)*pi/2
                    transformationMatrix = mat([
                        [cos(theta), sin(theta), xIndex*self.roadSegmentLength], 
                        [-sin(theta), cos(theta), yIndex*self.roadSegmentLength],
                        [0,0,1]])

                    for templIndex in range(len(self.vertsTemplate)):
                        print "xIndex: "+str(xIndex)+"   yIndex: "+str(yIndex)+"   thIndex: "+str(thIndex)
                        print (transformationMatrix[:2])
                        print (transformationMatrix[:2] * self.vertsTemplate[0].T)
                        print nextRegionVertices
                        error = linalg.norm((transformationMatrix[:2] * self.vertsTemplate[0].T) - nextRegionVertices)
                        print "error: "+str(error)
                        if error < 0.001:
                            return templIndex, transformationMatrix, thIndex


