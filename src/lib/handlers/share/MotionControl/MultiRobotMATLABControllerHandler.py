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
import copy

import lib.handlers.handlerTemplates as handlerTemplates

class MultiRobotMATLABControllerHandler(handlerTemplates.MotionControlHandler):
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

        # setup matlab communication
        self.session = MATLABPythonInterface.initializeMATLABPythonCommunication(self.rfi.regions, self.coordmap_map2lab)

        self.old_vx = None
        self.old_vy = None

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

            # Find our current configuration (Using pixels now)
            # pose.update([(robot_name, self.pose_handler[robot_name].getPose())])
            pose.update([(robot_name, \
                append(array(self.coordmap_lab2map(self.pose_handler[robot_name].getPose())), self.pose_handler[robot_name].getPose()[2]))])
            # Check if Vicon has cut out
            # TODO: this should probably go in posehandler?
            if math.isnan(pose[robot_name][2]):
                print "WARNING: No Vicon data! Pausing."
                for robot_name in self.robotList:
                    self.drive_handler[robot_name].setVelocity(0, 0)  # So let's stop
                time.sleep(1)
                # return False not leaving yet until all robots are checked

            # NOTE: Information about region geometry can be found in self.rfi.regions:
            ##### (using pixels now) ####
            # vertices = mat(map(self.coordmap_map2lab, [x for x in self.rfi.regions[current_reg].getPoints()])).T
            vertices = mat([x for x in self.rfi.regions[current_reg].getPoints()])
            current_regVertices[robot_name] = vertices

            # vertices = mat(map(self.coordmap_map2lab, [x for x in self.rfi.regions[next_reg].getPoints()])).T
            vertices = mat([x for x in self.rfi.regions[next_reg].getPoints()])
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
        logging.debug("current:" + str(current_regIndices.values()))
        logging.debug("next:" + str(next_regIndices.values()))
        if not current_regIndices.values() == next_regIndices.values():
            if self.temp_next is None or next_regIndices != self.temp_next:
                logging.info([int(x) + 1 for x in next_regIndices.values()])
                self.temp_next = copy.deepcopy(next_regIndices)
            try:
                vx, vy, regionChanges, currentLoc = MATLABPythonInterface.getMATLABVelocity(self.session, pose, next_regIndices)
                self.resetMATLAB = False
            except:
                logging.debug("caught exception")
                # time.sleep(10)
                if self.old_vx is None or self.old_vy is None:
                    for idx, robot_name in enumerate(self.robotList):
                        vx.append(0)
                        vy.append(0)
                else:
                    vx = self.old_vx
                    vy = self.old_vy
                regionChanges = array([])
                currentLoc = array([])

                # delete and restart MATLAB session
                MATLABPythonInterface.closeInterface(self.session)
                logging.info("Restarting the MATLAB interface and hopefully this will solve the problem")
                # time.sleep(10)
                self.session = MATLABPythonInterface.initializeMATLABPythonCommunication(self.rfi.regions, self.coordmap_map2lab)
                self.resetMATLAB = True

        else:
            logging.debug('Staying in place. Velocities are now set to zero.')
            vx = []
            vy = []
            regionChanges = array([])
            for idx, robot_name in enumerate(self.robotList):
                vx.append(0)
                vy.append(0)
            currentLoc = array([])
            self.resetMATLAB = False


        self.old_vx = vx
        self.old_vy = vy
        logging.debug("regionChanges:" + str(regionChanges) + " currentLoc:" + str(currentLoc))
        # check if we want a different region changes for now.
        for idx, robot_name in enumerate(self.robotList):
            if regionChanges.any():
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
                    self.current_regIndices[robot_name] = current_regIndices[robot_name]  # storing idx of decomposed regions
                logging.debug(robot_name + '-vx:' + str(vx[idx]) + ' vy:' + str(vy[idx]))
                # conduct mapping because from lab to map
                multiplier = 1.5
                if self.experimentInLab:
                    vy[idx] = -1 * vy[idx]

                self.drive_handler[robot_name].setVelocity(vx[idx] * multiplier, \
                                     vy[idx] * multiplier, pose[robot_name][2])

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
                    if is_inside([pose[robot_name][0], pose[robot_name][1]], vertices):
                        logging.info("I think I'm in " + r.name)
                        break
                self.last_warning = time.time()

        # logging.debug("arrived:" + str(arrived))
        return (True in arrived.values())  # arrived[self.executor.hsub.getMainRobot().name]
