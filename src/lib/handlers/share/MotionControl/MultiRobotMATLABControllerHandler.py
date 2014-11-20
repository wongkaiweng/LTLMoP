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
import __MATLABPythonInterface as MATLABPythonInterface
from collections import OrderedDict

import lib.handlers.handlerTemplates as handlerTemplates

class MultiRobotMATLABControllerHandler(handlerTemplates.MotionControlHandler):
    def __init__(self, executor, shared_data):
        """
        Vector motion planning controller
        """
        # get the list of robots
        self.robotList = [robot.name for robot in executor.hsub.executing_config.robots]

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
        self.last_warning = 0

        # setup matlab communication
        self.session = MATLABPythonInterface.initializeMATLABPythonCommunication(self.rfi.regions, self.coordmap_map2lab)

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

        pose = OrderedDict()
        for robot_name, current_reg in current_regIndices.iteritems():
            next_reg = next_regIndices[robot_name]

            # Find our current configuration
            pose.update([(robot_name, self.pose_handler[robot_name].getPose())])

            # Check if Vicon has cut out
            # TODO: this should probably go in posehandler?
            if math.isnan(pose[robot_name][2]):
                print "WARNING: No Vicon data! Pausing."
                self.drive_handler[robot_name].setVelocity(0, 0)  # So let's stop
                time.sleep(1)
                # return False not leaving yet until all robots are checked

            # NOTE: Information about region geometry can be found in self.rfi.regions:
            vertices = mat(map(self.coordmap_map2lab, [x for x in self.rfi.regions[current_reg].getPoints()])).T
            current_regVertices[robot_name] = vertices

            vertices = mat(map(self.coordmap_map2lab, [x for x in self.rfi.regions[next_reg].getPoints()])).T
            next_regVertices[robot_name] = vertices

            """
            if current_reg == next_reg and not last:
                logging.debug('stop moving, regions the same')
                # No need to move!
                self.drive_handler[robot_name].setVelocity(0, 0)  # So let's stop
                continue
                #return True not leaving until all robots are checked
            """
        ################################
        ### NORA's motion controller ###
        ################################

        # Run algorithm to find a velocity vector (global frame) to take the robot to the next region
        vx, vy, regionChanges, currentLoc = MATLABPythonInterface.getMATLABVelocity(self.session, pose, next_regIndices)

        # check if we want a different region changes for now.
        for idx, robot_name in enumerate(self.robotList):
            if regionChanges.any():
                departed[robot_name] = True
                arrived[robot_name] = True
                self.current_regIndices[robot_name] = currentLoc[idx]  # storing idx of decomposed regions
                self.executor.postEvent("INFO", "regionChanges corrected. current_regIdx:" + str(self.current_regIndices[robot_name]))
                time.sleep(3)
            else:
                # Figure out whether we've reached the destination region
                departed[robot_name] = False  # not is_inside([pose[robot_name][0], pose[robot_name][1]], current_regVertices[robot_name])
                arrived[robot_name] = False  # is_inside([pose[robot_name][0], pose[robot_name][1]], next_regVertices[robot_name])
                self.current_regIndices[robot_name] = current_regIndices[robot_name]  # storing idx of decomposed regions
                logging.debug(robot_name + '-vx:' + str(vx[idx]) + ' vy:' + str(vy[idx]))
                self.drive_handler[robot_name].setVelocity(vx[idx], vy[idx], pose[robot_name][2])

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
                    pointArray = [self.coordmap_map2lab(x) for x in r.getPoints()]
                    vertices = mat(pointArray).T
                    if is_inside([pose[robot_name][0], pose[robot_name][1]], vertices):
                        logging.info("I think I'm in " + r.name)
                        break
                self.last_warning = time.time()

        # logging.debug("arrived:" + str(arrived))
        return (True in arrived.values())  # arrived[self.executor.hsub.getMainRobot().name]
