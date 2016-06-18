#!/usr/bin/env python
"""
=======================================================
MultiRobotMATLABController.py - Vector Addition Motion Controller
=======================================================

Uses the vector field algorithm developed by Stephen R. Lindemann to calculate a global velocity vector to take the robot from the current region to the next region, through a specified exit face.
"""

from numpy import *
from __is_inside import *
import time, math
import logging
import __MATLABPythonInterface_direct as MATLABPythonInterface
import matlab.engine
from collections import OrderedDict
import copy
from multiprocessing.managers import BaseManager

import lib.handlers.handlerTemplates as handlerTemplates

class MultiRobotMATLABSingleRobotControllerHandler(handlerTemplates.MotionControlHandler):
    def __init__(self, executor, shared_data):
        """
        Vector motion planning controller
        """
        # get the list of robots
        self.robotList = [robot.name for robot in executor.hsub.executing_config.robots]
        logging.debug("robotList:" + str(self.robotList))
        self.drive_handler = {}
        self.pose_handler = {}
        self.current_regIndices = {}
        self.executor = executor

        # Get references to handlers we'll need to communicate with
        for robot_name in self.robotList:  # x must be a string
            self.drive_handler[robot_name] = executor.hsub.getHandlerInstanceByType(handlerTemplates.DriveHandler, robot_name)
            self.drive_handler[robot_name].loco = executor.hsub.getHandlerInstanceByType(handlerTemplates.LocomotionCommandHandler, robot_name)
            self.pose_handler[robot_name] = executor.hsub.getHandlerInstanceByType(handlerTemplates.PoseHandler, robot_name)
            self.current_regIndices[robot_name] = None

        # Get information about regions
        self.rfi = executor.proj.rfi
        self.coordmap_map2lab = executor.hsub.coordmap_map2lab
        self.coordmap_lab2map = executor.hsub.coordmap_lab2map  # use pixels now
        self.last_warning = 0

        # check if it's simulation or running in the lab
        robotTypeList = [robot.r_type for robot in executor.hsub.executing_config.robots]
        if "basicSim" in robotTypeList:
            self.experimentInLab = False
            logging.debug("In simulation")
        else:
            self.experimentInLab = True
            logging.debug("Running in the lab")
        
        self.resetMATLAB = False  # default reset to zero
        self.temp_next = None

        # NOTE: you need to first start __MATLABMultiRobotServer
        # setup matlab communication (single)
        self.session = matlab.engine.connect_matlab(matlab.engine.find_matlab()[0])
        #self.session = matlab.engine.connect_matlab('MATLAB_Nora')

        robot_name = self.robotList[0]
        pose = self.pose_handler[robot_name].getPose()

        for idx, r in enumerate(self.rfi.regions):
            # using pixels now
            pointArray = [self.coordmap_map2lab(x) for x in r.getPoints()]
            vertices = mat(pointArray).T
            #vertices = mat([x for x in r.getPoints()])
            if is_inside([pose[0], pose[1]], vertices):
                current_region = idx
                break
        logging.debug('Current region is :' + str(current_region))
        MATLABPythonInterface.init_singleRobot(self.session, robot_name, pose, current_region)

        # init robot
        self.initalized = False

        self.old_vx = None
        self.old_vy = None
        self.old_regionChanges = array([])
        self.old_next_regIndices = array([])
        self.old_currentLoc = array([])

    def gotoRegion(self, current_regIndices, next_regIndices, last=False):
        """
        If ``last`` is True, we will move to the center of the destination region.

        Returns ``True`` if we've reached the destination region.

        current_regAllIndices: dictionary of region indices
        next_regAllIndices: dictionary of region indices
        """

        current_regVertices = {}
        next_regVertices = {}
        pose = {}
        departed = {}
        arrived = {}
        V = {}

        # logging.debug("current_regIndices:" + str(current_regIndices))
        # logging.debug("next_regIndices: " + str(next_regIndices))

        robot_name = self.robotList[0]
        pose = self.pose_handler[robot_name].getPose()

        if math.isnan(pose[2]):
            print "WARNING: No Vicon data! Pausing."
            for robot_name in self.robotList:
                self.drive_handler[robot_name].setVelocity(0, 0)  # So let's stop
        ###################
        ## init session ###
        ###################
        #if not self.initalized:
        #    MATLABPythonInterface.init_singleRobot(self.session, robot_name, pose, next_regIndices[0])
        #    time.sleep(10)
        #    self.initalized = True
        #    logging.debug("pose: {pose}".format(pose=pose))


        ################################
        ### NORA's motion controller ###
        ################################
        logging.debug("current:" + str(current_regIndices.values()) + ", next:" + str(next_regIndices.values()))
        logging.debug("self.old_regionChanges:" + str(self.old_regionChanges))
        logging.debug("self.old_currentLoc:" + str(self.old_currentLoc) + 'current_regIndices:' + str(current_regIndices) + 'self.current_regIndices:' + str(self.current_regIndices))
        logging.debug("self.old_next_regIndices:" + str(self.old_next_regIndices) + " next_regIndices:" + str(next_regIndices))
        vx = []
        vy = []
        regionChanges = []
        currentLoc = []
        vx, vy, regionChanges, currentLoc = MATLABPythonInterface.getSingleMATLABVelocity(self.session, robot_name, pose, next_regIndices[robot_name])


        self.old_vx = array(vx)
        self.old_vy = array(vy)
        self.old_regionChanges = array(regionChanges)
        self.old_next_regIndices = copy.deepcopy(next_regIndices)
        self.old_currentLoc = array(currentLoc)
        logging.debug('vx:' + str(vx) + ' vy:' + str(vy))
        logging.debug("regionChanges:" + str(regionChanges) + " currentLoc:" + str(currentLoc))
        if regionChanges:
            logging.info("current region:" + str(currentLoc))

        if not vx:
            vx = [0,0,0]

        if not vy:
            vy = [0,0,0]


        # check if we want a different region changes for now.
        idx = MATLABPythonInterface.get_robot_list().index(robot_name)

        if regionChanges and regionChanges[idx] != regionChanges[idx]:
            departed[robot_name] = True
            arrived[robot_name] = True
            self.current_regIndices[robot_name] = currentLoc[idx]  # storing idx of decomposed regions
            # self.executor.postEvent("INFO", "regionChanges corrected. current_regIdx:" + str(self.current_regIndices[robot_name]))
            # time.sleep(3)
        else:
            # Figure out whether we've reached the destination region
            departed[robot_name] = False  # not is_inside([pose[robot_name][0], pose[robot_name][1]], current_regVertices[robot_name])
            arrived[robot_name] = False  # is_inside([pose[robot_name][0], pose[robot_name][1]], next_regVertices[robot_name])
            if self.resetMATLAB:
                self.current_regIndices[robot_name] = None
            else:
                if currentLoc:
                    self.current_regIndices[robot_name] = currentLoc[idx]
                else:
                    self.current_regIndices[robot_name] = current_regIndices[robot_name]  # storing idx of decomposed regions

            logging.debug(robot_name + '-vx:' + str(vx[idx]) + ' vy:' + str(vy[idx]))

            # conduct mapping because from lab to map
            multiplier = 1.5
            if self.experimentInLab:
                vy[idx] = -1 * vy[idx]

            self.drive_handler[robot_name].setVelocity(vx[idx] * multiplier, \
                                 vy[idx] * multiplier, pose[2])

        # logging.debug("current_regVertices[robot_name]"+str(current_regVertices[robot_name]))
        # logging.debug("departed[robot_name]" + str(departed[robot_name]))
        # logging.debug("arrived[robot_name]" + str(arrived[robot_name]))

        # logging.debug("V:"+ str(V))
        # OUTPUT from Nora's motion control
        # for example: V = {'robot1':[1,2,3],'robot2':[4,5,6]}

        if departed[robot_name] and (not arrived[robot_name]) and (time.time() - self.last_warning) > 0.5:
            # print "WARNING: Left current region but not in expected destination region"
            # Figure out what region we think we stumbled into
            for r in self.rfi.regions:
                # using pixels now
                # pointArray = [self.coordmap_map2lab(x) for x in r.getPoints()]
                # vertices = mat(pointArray).T
                vertices = mat([x for x in r.getPoints()])
                if is_inside([pose[0], pose[1]], vertices):
                    logging.info("I think I'm in " + r.name)
                    break
            self.last_warning = time.time()

        logging.debug("arrived:" + str(arrived))
        logging.debug('#######################################################')
        return arrived[robot_name]  # arrived[self.executor.hsub.getMainRobot().name]
