#!/usr/bin/env python
"""
=====================================
dummySensor.py - Dummy Sensor Handler
=====================================

Displays a silly little window for faking sensor values by clicking on buttons.
"""

import threading, subprocess, os, time, socket
import numpy, math
import sys
import ast, select
# ---- two_robot_negotiation  --- #
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

import random
import Polygon, Polygon.IO, Polygon.Utils, Polygon.Shapes
from copy import deepcopy

# Climb the tree to find out where we are
p = os.path.abspath(__file__)
t = ""
while t != "share":
    (p, t) = os.path.split(p)
    if p == "":
        print "I have no idea where I am; this is ridiculous"
        sys.exit(1)
ltlmop_logger.debug(p)
sys.path.append(os.path.join(p,"share","Pose"))
sys.path.append(os.path.join(p,"share","MotionControl"))
import _pyvicon
# ------------------------------- #

import lib.handlers.handlerTemplates as handlerTemplates

from __is_inside import is_inside

class DummySensorHandler(handlerTemplates.SensorHandler):
    def __init__(self, executor, shared_data):
        """
        Start up sensor handler subwindow and create a new thread to listen to it.
        """

        # Since we don't want to have to poll the subwindow for each request,
        # we need a data structure to cache sensor states:
        self.sensorValue = {}
        if executor:
            self.proj = executor.proj
        self.executor = executor
        self.sensorListenInitialized = False
        self._running = True
        self.p_sensorHandler = None
        self.port = random.randint(10000, 65535)         #port = 23459
        
        # --- two_robot_negotiation --- #
        self.robClient = None # fetch negMonitor from executor 
        if executor:
            ltlmop_logger.debug(executor.robClient)
        self.robotRegionStatus  = {} # for keeping track of robot locations
        self.viconServer = {} # dict of vicon poses
        self.prev_pose = [] # storing prev pose
        self.currentRegionPoly = None # current region polygon
        self.prev_current_region = '' # storing prev region str
        self.polyRegionList = {} # region polygon dict
        self.radius = 5 # radius of robot poly
        self.robotLocationsCopy = {} # copy of the dPatching robotLocations dict
        self.actionStatusCopy = {} # copy of the dPatching actionStatus dict
        self.robotSensorInfo = {} # sensor info dict
        # ----------------------------- #

        self.dummyActuatorHandler = None
        self.sock = {}
        self.boolValue = {}

    def _stop(self):
        if self.p_sensorHandler is not None:
            print >>sys.__stderr__, "(SENS) Killing dummysensor GUI..."
            self.p_sensorHandler.stdin.write(":QUIT\n")
            self.p_sensorHandler.stdin.close()

            print >>sys.__stderr__, "(SENS) Terminating dummysensor GUI listen thread..."
            self._running = False
            self.sensorListenThread.join()
        
        # --- two_robot_negotiation --- #
        
        # ----------------------------- #

    def _createSubwindow(self):
            # Create a subprocess
            self.executor.postEvent("INFO", "(SENS) Starting sensorHandler window and listen thread...")
            self.p_sensorHandler = subprocess.Popen([sys.executable, "-u", os.path.join(self.proj.ltlmop_root,"lib","handlers","share","Sensor","_SensorHandler.py"), str(self.port)], stdin=subprocess.PIPE)

            # Create new thread to communicate with subwindow
            self.sensorListenThread = threading.Thread(target = self._sensorListen)
            self.sensorListenThread.daemon = True
            self.sensorListenThread.start()

            # Block until the sensor listener gets the go-ahead from the subwindow
            while not self.sensorListenInitialized:
                time.sleep(0.05) # Yield cpu

    def regionBit(self,name,init_region,bit_num,initial=False):
        """
        Return the value of bit #bit_num in the bit-vector encoding of the currently selected region

        name (string): Unique identifier for region sensor (default="target")
        init_region (region): Name of the sensor whose state is interested
        bit_num (int): The index of the bit to return
        """
        if initial:
            if not self.sensorListenInitialized:
                self._createSubwindow()

            if name not in self.sensorValue.keys():
                # create a new map element
                # choose an initial (decomposed) region inside the desired one
                self.sensorValue[name] = self.proj.regionMapping[init_region][0]
                self.p_sensorHandler.stdin.write("loadproj," + self.proj.getFilenamePrefix() + ".spec,\n")
                self.p_sensorHandler.stdin.write(",".join(["region", name, self.sensorValue[name]]) + "\n")
            return True
        else:
            if name in self.sensorValue:
                reg_idx = self.proj.rfi.indexOfRegionWithName(self.sensorValue[name])
                numBits = int(math.ceil(math.log(len(self.proj.rfi.regions),2)))
                reg_idx_bin = numpy.binary_repr(reg_idx, width=numBits)
                #print name, bit_num, (reg_idx_bin[bit_num] == '1')
                return (reg_idx_bin[bit_num] == '1')
            else:
                self.executor.postEvent("INFO", "(SENS) WARNING: Region sensor %s is unknown!" % button_name)
                return None

    def buttonPress(self,button_name,init_value,initial=False):
        """
        Return a boolean value corresponding to the state of the sensor with name ``sensor_name``
        If such a sensor does not exist, returns ``None``

        button_name (string): Name of the sensor whose state is interested
        init_value (bool): The initial state of the sensor (default=False)
        """
        if initial:

            if not self.sensorListenInitialized:
                self._createSubwindow()

            if button_name not in self.sensorValue.keys():
                self.sensorValue[button_name] = init_value
                if init_value:
                    self.p_sensorHandler.stdin.write("button," + button_name + ",1\n")
                else:
                    self.p_sensorHandler.stdin.write("button," + button_name + ",0\n")
            return self.sensorValue[button_name]
        else:
            if button_name in self.sensorValue:
                return self.sensorValue[button_name]
            else:
                self.executor.postEvent("INFO", "(SENS) WARNING: Sensor %s is unknown!" % button_name)
                return None

    def _sensorListen(self):
        """
        Processes messages from the sensor handler subwindow, and updates our cache appropriately
        """
        host = 'localhost'
        buf = 1024
        addr = (host, self.port)

        UDPSock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        UDPSock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        UDPSock.settimeout(1)
        try:
            UDPSock.bind(addr)
        except:
            print "ERROR: Cannot bind to port.  Try killing all Python processes and trying again."
            return

        while self._running:
            # Wait for and receive a message from the subwindow
            try:
                input,addrFrom = UDPSock.recvfrom(1024)
            except socket.timeout:
                continue

            if input == '':  # EOF indicates that the connection has been destroyed
                self.executor.postEvent("INFO", "(SENS) Sensor handler listen thread is shutting down.")
                break

            # Check for the initialization signal, if necessary
            if not self.sensorListenInitialized and input.strip() == "Hello!":
                self.sensorListenInitialized = True
                continue

            # Get the data out of the message
            args = input.strip().split("=")

            if len(args) != 2:
                continue

            # Update our internal cache
            if args[1] == "True":
                self.sensorValue[args[0]] = True
            elif args[1] == "False":
                self.sensorValue[args[0]] = False
            else:
                self.sensorValue[args[0]] = args[1]

    # ----- two_robot_negotiation ---- #
    def _requestRegionInfo(self, initial = False):
        """
        This function update the region values from negMonitor.
        """
        if self.robClient is None:
            self.robClient = self.executor.robClient # fetch robClient from executor 

        self.robotRegionStatus = self.robClient.requestRegionInfo()

        #overloaded a bit. also request sensors info
        self.robotSensorInfo = self.robClient.getRobotSensorsStatus()

    def _lockCurrentRegion(self, initial=False):
        """
        This funcion locks the current region pose such that mutual exclusion is enforced.
        """
        if self.proj.rfi:
            pose =self.executor.hsub.getPose()
            #########################################
            ### Copied from vectorController.py #####
            #########################################

            # make sure the pose is valid
            if sum(pose) == 0:
                pose = self.prev_pose # maybe do interpolation later?
                ltlmop_logger.warning("Losing pose... Using old one.")
            else:
                self.prev_pose = pose

            # form polygon for the robot
            RobotPoly = Polygon.Shapes.Circle(1.2*self.radius,(pose[0],pose[1]))

            # check robot location
            for regionName in self.polyRegionList.keys():
                if self.polyRegionList[regionName].covers(RobotPoly):
                    self.currentRegionPoly = self.polyRegionList[regionName]
                    if regionName != self.prev_current_region:
                        ltlmop_logger.debug("Dummy SENSOR: current region changed to:" + str(regionName))
                    self.prev_current_region = regionName

            if not self.currentRegionPoly.overlaps(RobotPoly):
                ltlmop_logger.warning("not inside next region or overlaps current region?!")

        # also lock locations of environment robots
        if self.executor.proj.compile_options['neighbour_robot'] and self.executor.proj.compile_options["multi_robot_mode"] == "d-patching":
            if self.executor.dPatchingExecutor.robotLocations:
                while self.executor.dPatchingExecutor.regionLock.locked():
                    time.sleep(0.002)
                self.robotLocationsCopy = deepcopy(self.executor.dPatchingExecutor.robotLocations)

            if self.executor.dPatchingExecutor.actionStatus:
                while self.executor.dPatchingExecutor.actionLock.locked():
                    time.sleep(0.002)
                self.actionStatusCopy = deepcopy(self.executor.dPatchingExecutor.actionStatus)

    def otherRobotActionStatus(self, robot_name, action, initial = False):
        """
        get action status from patching executor
        robot_name (string): name of the robot
        action (string): action name
        """
        try:
            if self.executor.proj.compile_options["multi_robot_mode"] == "d-patching":
                return self.actionStatusCopy[robot_name][action]
            else:
                ltlmop_logger.warning('not matching any mode in dummy. returning None.')
                return None
        except:
            #ltlmop_logger.info('Variable' + region + ',' +  robot_name + ' is not initialized yet!')
            return None

    def inRegion(self, regionName, radius, initial=False):
        """
        Check if the robot is in this region
        regionName (string): Name of the region
        radius (float): radius of the robot, 5.0 for basicSim and 0.15 for Nao (default=5.0)
        """
        if initial:
            self.prev_pose = self.executor.hsub.getPose()

            self.polyRegionList[regionName] = Polygon.Polygon()
            decomposedRegionList = self.proj.regionMapping[regionName]
            for region in decomposedRegionList:
                regionNo = self.proj.rfi.indexOfRegionWithName(region)
                #ltlmop_logger.debug('regionNo:{0}'.format(regionNo))
                pointArray = [x for x in self.proj.rfi.regions[regionNo].getPoints()]
                pointArray = map(self.executor.hsub.coordmap_map2lab, pointArray)
                self.polyRegionList[regionName] += Polygon.Polygon([(pt[0],pt[1]) for pt in pointArray])
                #ltlmop_logger.debug('regionName {0} : {1}'.format(decomposedRegionList, self.polyRegionList[regionName]))
                #ltlmop_logger.debug('holeList length : {0}'.format(len(self.proj.rfi.regions[regionNo].holeList)))

                if len(self.proj.rfi.regions[regionNo].holeList):
                    for hole_idx in range(len(self.proj.rfi.regions[regionNo].holeList)):
                        pointArray = [x for x in self.proj.rfi.regions[regionNo].getPoints(hole_id=hole_idx)]
                        pointArray = map(self.executor.hsub.coordmap_map2lab, pointArray)
                        self.polyRegionList[regionName] -= Polygon.Polygon([(pt[0],pt[1]) for pt in pointArray])

            if self.polyRegionList[regionName].isInside(self.prev_pose[0], self.prev_pose[1]):
                self.prev_current_region = regionName
                self.currentRegionPoly = self.polyRegionList[regionName]
            self.radius = radius

        else:
            if regionName == self.prev_current_region:
                return True
            else:
                return False

    def inRegion_old(self, regionName , initial = False):
        """
        Check if the robot is in this region
        regionName (string): Name of the region
        """

        if initial:
            self.prev_pose = self.executor.hsub.getPose()
            return True

        else:
            pose =self.executor.hsub.getPose()
            #########################################
            ### Copied from vectorController.py #####
            #########################################

            # make sure the pose is valid
            if sum(pose) == 0:
                pose = self.prev_pose # maybe do interpolation later?
                ltlmop_logger.warning("Losing pose... Using old one.")
            else:
                self.prev_pose = pose

            regionNo = self.proj.rfiold.indexOfRegionWithName(regionName)
            pointArray = [x for x in self.proj.rfiold.regions[regionNo].getPoints()]
            pointArray = map(self.executor.hsub.coordmap_map2lab, pointArray)
            vertices = numpy.mat(pointArray).T
            #ltlmop_logger.debug('self.proj.rfiold.regions:' + str(self.proj.rfiold.regions))
            #ltlmop_logger.debug("pose:" + str(pose))
            #ltlmop_logger.debug("vertices:" + str(vertices))
            #ltlmop_logger.debug(self.proj.rfiold.regions[regionNo].name +": " +  str(is_inside([pose[0], pose[1]], vertices)))
            return is_inside([pose[0], pose[1]], vertices)

    def otherRobotSensorStatus(self, robot_name, sensor_name, initial=False):
        """
        request other robot's sensors from negotiation Monitor. For negotiation only
        robot_name (string): name of the robot
        sensor_name (string): sensor name
        """
        try:
            if self.executor.proj.compile_options["multi_robot_mode"] == "patching" or self.executor.proj.compile_options["multi_robot_mode"] == "negotiation":
                return self.robotSensorInfo[robot_name][sensor_name]
            elif self.executor.proj.compile_options["multi_robot_mode"] == "d-patching":
                ltlmop_logger.warning('This function is not supported for d-patching')
            else:
                ltlmop_logger.warning('not matching any mode in dummy. returning None.')
                return None
        except:
            #ltlmop_logger.info('Variable' + region + ',' +  robot_name + ' is not initialized yet!')
            return None

    def otherRobotLocation(self, robot_name, region, initial = False):
        """
        request other robot's location from negotiation Monitor.
        robot_name (string): name of the robot
        region (string): region name
        """
        try:
            if self.executor.proj.compile_options["multi_robot_mode"] == "patching" or self.executor.proj.compile_options["multi_robot_mode"] == "negotiation":
                #ltlmop_logger.info(robot_name + '-' + region + ': ' + str(self.robotRegionStatus[region][robot_name]))
                return self.robotRegionStatus[region][robot_name]
            elif self.executor.proj.compile_options["multi_robot_mode"] == "d-patching":
                return self.robotLocationsCopy[region][robot_name]
            else:
                ltlmop_logger.warning('not matching any mode in dummy. returning None.')
                return None
        except:
            #ltlmop_logger.info('Variable' + region + ',' +  robot_name + ' is not initialized yet!')
            return None


    def checkIfViconObjectNearLocation(self, vicon_object_name, poseX, poseY, range=0.5, initial=False):
        """
        check if vicon object is near location specified.
        vicon_object_name (string): name of the vicon object (default='folder:mainBody')
        poseX (float): x-coorindate of the location (default=0.0)
        poseY (float): y-coorindate of the location (default=0.0)
        """
        if initial:
            print "Connecting to Vicon server..."
            self.viconServer[vicon_object_name] = _pyvicon.ViconStreamer()
            self.viconServer[vicon_object_name].connect("10.0.0.102", 800)

            #model_name = "GPSReceiverHelmet-goodaxes:GPSReceiverHelmet01"
            #model_name = "folder:mainBody"

            self.viconServer[vicon_object_name].selectStreams(["Time"] + ["{} <{}>".format(vicon_object_name, s) for s in ("t-X", "t-Y")])
            self.viconServer[vicon_object_name].startStreams()

            # Wait for first data to come in
            while self.viconServer[vicon_object_name].getData() is None:
                pass

        else:
            (t, x, y) = self.viconServer[vicon_object_name].getData()
            (t, x, y) = [t/100, x/1000, y/1000]

            # Find our current configuration
            pose = [poseX/1000, poseY/1000]

            #if math.sqrt((pose[0]-x)**2+(pose[1]-y)**2) < range:
                #print >>sys.__stdout__, "See" + vicon_object_name + ": location: " + str(pose) + vicon_object_name +": " + str(x) + str(y)  + "range: " + str(math.sqrt((pose[0]-x)**2+(pose[1]-y)**2))

            return math.sqrt((pose[0]-x)**2+(pose[1]-y)**2) < range

    def imageDisplayCompletion(self, actuatorName, initial=False):
        """
        check if the image display action is completed
        actuatorName (string): name of the corresponding actuator
        """
        if initial:
            if not self.dummyActuatorHandler:
                self.dummyActuatorHandler = self.executor.hsub.getHandlerInstanceByName("DummyActuatorHandler")
        else:
            #if self.dummyActuatorHandler.imageDisplayCompletionStatus[actuatorName]:
            #    ltlmop_logger.log(6, self.dummyActuatorHandler.imageDisplayCompletionStatus[actuatorName])
            return self.dummyActuatorHandler.imageDisplayCompletionStatus[actuatorName]

    def button_state(self, actuatorName, initial=False):
        """
        check if the image display action is completed
        actuatorName (string): name of the corresponding actuator
        """
        if initial:
            if not self.dummyActuatorHandler:
                self.dummyActuatorHandler = self.executor.hsub.getHandlerInstanceByName("DummyActuatorHandler")
        else:
            #if self.dummyActuatorHandler.imageDisplayCompletionStatus[actuatorName]:
            #    ltlmop_logger.log(6, self.dummyActuatorHandler.imageDisplayCompletionStatus[actuatorName])
            return self.dummyActuatorHandler.buttonPressCompletionStatus[actuatorName]

    def checkBroadcast(self, port, initialValue, initial=False):
        """
        This function checks broadcasting msg.
        port (int): port number (default=12345)
        initialValue (bool): initial expected bool. (default=False)
        """
        if initial:
            self.sock[port] = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock[port].setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock[port].bind(('', port))
            self.sock[port].setblocking(0)
            self.boolValue[port] = initialValue
        else:
            #print hasattr(self.sock, 'select')
            ready = select.select([self.sock[port]], [], [],0)
            if ready[0]:
                self.boolValue[port] = ast.literal_eval(self.sock[port].recv(4096))
                ltlmop_logger.debug("{0} - valueChanged: {1}".format(port, self.boolValue[port]))
                self.executor.postEvent("INFO", "(SENS) Port {0} changed to {1}".format(port, self.boolValue[port]))
            return self.boolValue[port]

if __name__ == "__main__":
    sen = DummySensorHandler(None, None)
    sen.checkBroadcast(12345, False, initial=True)
    while True:
        sen.checkBroadcast(12345, True, initial=False)
