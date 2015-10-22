#!/usr/bin/env python
"""
This module handles communication among robots and also the execution of global automaton.
"""
import socket #import the socket library
import logging
import select #for multiple robot client connections
import re     #for parsing msg from client
import ast    #for parsing msg from client
import sys    #for program exiting
import signal #for terminating object when keyboard interrupt
import time   #for temporary pause, timing synthesis time
import random #for chosing one next states from the list
import Queue  #for queueing messages from other robots
import copy   #for tracking coorindating robots
import threading #for monitoring sysGoals

# Climb the tree to find out where we are
import os
p = os.path.abspath(__file__)
t = ""
while t != "src":
    (p, t) = os.path.split(p)
    if p == "":
        print "I have no idea where I am; this is ridiculous"
        sys.exit(1)

sys.path.append(os.path.join(p, "src", "lib"))

#for compiling the centralized spec
import specCompiler
from createJTLVinput import createLTLfile, createSMVfile
import strategy
import LTLParser.LTLcheck

# recursionlimits
logging.debug("recursion limits:" + str(sys.getrecursionlimit()))
sys.setrecursionlimit(1500)

from decentralizedPatchingMSGHelper import MsgHandlerExtensions #also inherit other functions

##let's set up some constants
BUFSIZE = 20000    #reasonably sized buffer for data

def printRegionInfo(robotLocations):
    """
    This function prints the current region information of each robot
    """
    logging.info('----------------------------------------------')
    for region, robots in robotLocations.iteritems():
        table = ""
        for rob, status in robots.iteritems():
            table = table + "-{0:10}: {1:6} ".format(rob, status)
        table = "{0:13}".format(region) + table
        logging.info(table)
    logging.info('----------------------------------------------')

def printSpec(specType, specStr, robotName):
    """
    This function prints the spec type and the spec.
    specType: specification type
    specStr : specification string
    robotName: name of the robot that has this specification
    """
    logging.info('===============================================')
    logging.info('==== ' + specType + ' of ' + robotName + ' ====')
    logging.info('===============================================')
    logging.info(specStr)
    logging.info('===============================================')


class PatchingExecutor(MsgHandlerExtensions, object):
    """
    LTLMoP robot object used to communicate with the negotiationMonitor
    """
    def __init__(self, hsub, proj, current_region=None, current_region_completed=None):
        """
        current_region: current region object of the robot
        current_region_completed: current region completed object of the robot
        """
        self.proj = proj
        self.hsub = hsub
        self.initializeVariables() #initialize permanent variables
        self.cleanVariables(first_time=True) #initialize variables that are cleaned after patching

        super(PatchingExecutor, self).__init__(hsub, proj)

        self.strategy = None # for the loaded new centralized strategy
        self.filePath = "/".join([os.path.dirname(os.path.abspath(__file__)), self.robotName + 'centralizedSpec']) #output SMV, LTL file directory and name
        self.keepConnection = True

        # obtain robot addresses
        self.robotAddresses = {}
        with open(os.path.join(p, "src", "lib", "centralCoordinator", "robotAddresses.txt"), 'r') as f:
            for line in f:
                self.robotAddresses[line.split(':')[0].strip()] = ast.literal_eval(line.split(':')[1].strip())
        f.closed

        # first find out our address
        ADDR = self.robotAddresses[self.robotName]    #we need a tuple for the address

        ## now we create a new socket object (serv)
        self.serv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        listenConn = 5  #listenConn is the maximum number of queued connections we'll allow

        #find our socket to the address
        self.serv.bind((ADDR))    #the double parens are to create a tuple with one element
        self.serv.listen(listenConn)
        logging.info('CONNECTION: Listening...')
        logging.debug('sockName:' + str(self.serv.getsockname())) # client's connection addr
        logging.debug('ADDR:' + str(ADDR))

        self.clients = {}
        self.clients[self.robotName] = self.serv    #self.clients = [self.serv]
        self.message_queues = {}                    # Outgoing message queues (socket:Queue)

        # also try communication with other robots
        for robot, robotADDR in self.robotAddresses.iteritems():
            if robot != self.robotName:
                try:
                    logging.info('CONNECTION: Connecting to ' + robot + ' with address ' + str(robotADDR))
                    csock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)   # Create a TCP/IP socket
                    csock.connect(robotADDR)
                    logging.info('CONNECTION: Connection to '+ robot + ' is successful!')
                    self.clients[robot] = csock

                    # for tempMsg
                    self.tempMsg[csock] = ""

                    # Give the connection a queue for data we want to send
                    self.message_queues[csock] = Queue.Queue()

                    #send out initial info
                    self.initializeRegionExchange(csock, current_region)
                    if self.proj.compile_options['include_heading']:
                        self.initializeCompletedRegionExchange(csock, current_region_completed)

                except:
                    logging.error('CONNECTION: Cannot connect to ' + robot + ' with address ' + str(robotADDR))

        # save current regions of the robot
        self.current_region = current_region
        self.current_region_completed = current_region_completed

        # start running the executor!
        #self.run()

    def initializeVariables(self):
        """
        # This function initialize variables that will not be reset after central patching.
        """
        # variables of myself
        self.robotName = ""  # name of myself
        self.regionList = [] # region list of myself
        self.regions = []  # region object list from proj of myself
        self.newRegionNameToOld = {}  #find out mapping from new_names to old ones of myself
        self.regionCompleted_domain = None #regionCompleted strategy domain of myself
        self.region_domain = None   #region strategy doamin of myself
        self.tic = time.time() # for checking sysGoals in centralized execution
        self.toc = time.time() # for checking sysgoals in centralized execution
        self.current_region = None # store the current region in the strategy (updated in updateRobotRegion)
        self.current_region_completed = None # store the current completed region in the strategy (updated in updateRobotRegionCompleted)

        # variables related to the other robots
        self.tempNonameSock = [] # for temprarily storing socks not matching any robot name
        self.prev_outputs = None    # storing aut execution outputs
        self.specRequestFromOther = [] #track if spec is requested (list of spec requested)
        self.robotLocations = {}  #tracking region info for each robot
        self.tempMsg = {} # temporarily save incomplete msg from robots. clear when used.
        self.checkedRestartStatus = False # track if all robots has checked restart status
        self.robotInRange = [] # list of robots that are in range (We are communicating with).
        self.robotStatusOnCentralizedStrategyExecution = {} # dict of robots ready to execute. False stands for not ready. True stands for ready.
        self.tempRobotStatusOnCentralizedStrategyExecution = {} # temporarily store robotStatusOnCentralizedStrategyExecution of other robots and update in checkIfOtherRobotsAreReadyToExecuteCentralizedStrategy. (Prevents incorrect removal of status)
        self.checkSysGoalsThread = None # for checking sysGoals in the background

    def cleanVariables(self, first_time=True):
        """
        This function clean and initialize all variables when patching is done/ when the instance is first created
        """
        self.spec = {'EnvInit' : {}, 'EnvTrans' : {}, 'EnvGoals' : {}, 'SysInit' : {}, 'SysTrans' : {}, 'SysGoals' : {}}
        self.sysGoalsOld = {} # for checking if goals are reached in patching
        self.winPos = {} # for storing the winning position formula of the robots
        self.coordinatingRobots = [] #track the robots cooridnating
        self.old_coordinatingRobots = [] # store previous copy of coordinatingRobots
        self.envPropList = {} #tracking env propositions of each robot envPropList[robot]= propList = {prop:value}
        self.sysPropList = {} #tracking sys propositions of each robot sysPropList[robot]= propList = {prop:value}
        self.propMappingNewToOld = {} #track mapping from newPropName back to oldPropName propMappingNewToOld[robot][newPropName] = oldPropName
        self.propMappingOldToNew = {} #track mapping from newPropName back to oldPropName propMappingOldToNew[robot][oldPropName] = newPropName
        self.smvSysPropList = [] #store newPropName for system props
        self.smvEnvPropList = [] #store newPropName for environment props
        self.currentState = None #state object. store the combined current state of all robots
        self.currentAssignment = {} # dict. store assignments of all props and all robots
        self.sensor_state = None # tracking next inputs for runtime monitoring
        self.coordinationRequest = {} #track if coorindation is initiated. True if started and false otherwise
        self.coordinationRequestSent = [] #track whom we have sent coordination request to.
        self.last_next_states = [] #track the last next states in our autonmaton execution
        self.sysGoalsCheck = {} # runtime monitoring object to check if goals are reached
        self.sysGoalsCheckStatus = {} # True if that goals had been satisfied. False otherwise
        self.goalsSatisfied = False # track if all goals are satisfied at least once.
        self.winPosCheck = None # runtime monitoring object to check if goals are reached
        self.nextPossibleStatesArray = {} # store array of next possible states dict of robots. To be request by the other robots
        self.readyToRestart = {} # track if the two robots are ready to restart execution

        self.centralizedExecutionStatus = None # track centralized execution. True for centralized execution. False for waiting to execute centralized strategy. None for no centralized execution/execution ended.

        if not first_time:
            for robot_name in self.robotLocations.values()[0].keys():
                self.dictKeyFiller(robot_name)

    def dictKeyFiller(self, robot_name):
        """ This function add a dict key with the robot_name """

        # set patching status to false
        self.coordinationRequest[robot_name] = False

        # set up spec for the robot
        for specType in self.spec.keys():
            self.spec[specType][robot_name] = ""

        # set up propMappingNewToOld for the robot
        self.propMappingNewToOld[robot_name] = {}
        self.propMappingOldToNew[robot_name] = {}

        # set up nextPossibleStatesArray
        self.nextPossibleStatesArray[robot_name] = []

        # add robot key in sysGoalsOld
        self.sysGoalsOld[robot_name] = ""

        # add robot key to winPos
        self.winPos[robot_name] = ""

    def checkData(self):
        """
        Check if data is received and update info.
        """
        inputReady, outputReady, exceptReady = select.select(self.clients.values(), self.clients.values(), self.clients.values(), 0)

        for x in inputReady:

            if x == self.serv:
                csock, addr = self.serv.accept()
                csock.setblocking(0)
                logging.debug('peerName:' + str(csock.getpeername())) # my local connection addr
                logging.debug('sockName:' + str(csock.getsockname())) # client's connection addr
                if csock.getpeername() in self.robotAddresses.values():
                    acceptedRobotName = self.robotAddresses.keys()[self.robotAddresses.values().index(csock.getpeername())]
                    logging.info('CONNECTION: Accepted connection from ' + acceptedRobotName + ' with address '+ str(csock.getpeername()))
                    self.clients[acceptedRobotName] = csock
                else:
                    self.tempNonameSock.append(csock)
                    self.clients[csock] = csock
                    logging.info('CONNECTION: Accepted connection with address '+ str(csock.getpeername()) + '. Need to figure out robot name.')

                # for tempMsg
                self.tempMsg[csock] = ""

                # Give the connection a queue for data we want to send
                self.message_queues[csock] = Queue.Queue()

                #send out initial info, send current region info of myself to the other robot
                self.initializeRegionExchange(csock, self.current_region)

                # also update completed region
                if self.proj.compile_options['include_heading']:
                    self.initializeCompletedRegionExchange(csock, self.current_region_completed)

            else:
                # find data pattern
                data = x.recv(BUFSIZE)
                # retrieve incomplete data if there's any
                data = self.tempMsg[x]+data
                self.tempMsg[x] = ""

                # parse msg
                pattern = "(?P<robotName>\w+)-(?P<packageType>\w+)\s=\s(?P<packageValue>\[?.+\]?)\n"      # MSG FORMAT
                result = re.finditer(pattern, data)

                # temporarily save incomplete msg
                if not re.match(pattern, data):
                    self.tempMsg[x] = data
                else: # we have some patterns matching but the last one might be incomplete
                    self.tempMsg[x] = data[data.rfind('\n')+1:]

                for item in result:
                    # append sock to corresponding robotAddresses and remove it from self.tempNonameSock
                    if x in self.tempNonameSock and result:
                        self.robotAddresses[item.group("robotName")] = x.getpeername()
                        self.tempNonameSock.remove(x)
                        self.clients[item.group("robotName")] = x
                        del self.clients[x]

                    if item.group('packageType') == "regionNames":
                        # save region info and parse list into a dict of the form  robotLocations[r1][robot] = true

                        # convert region list string to list
                        rList = ast.literal_eval(item.group("packageValue"))

                        # first include the new robot in the old list of regions
                        for reg in self.robotLocations.keys():
                            if reg not in rList.keys():
                                self.robotLocations[reg][item.group("robotName")] = False

                        # store regions of the new robot be all false
                        for reg, value in rList.iteritems():
                            if not reg in self.robotLocations.keys():
                                self.robotLocations[reg] = {}
                                # also make sure other robots who doesn't have the regions have set that to false as well.
                                for robot in self.robotInRange:
                                    self.robotLocations[reg][robot] = False
                            self.robotLocations[reg][item.group("robotName")] = value

                        printRegionInfo(self.robotLocations)

                        # add new robot to our communication list
                        self.robotInRange.append(item.group("robotName"))

                        # initailize dict with new robot
                        self.dictKeyFiller(item.group("robotName"))

                    elif item.group('packageType') == "robotLocation":
                        # first figure out if it's rc region or not
                        RCregion = False
                        if item.group("packageValue").rfind('_rc') > 0:
                            RCregion = True

                        # update region info of robot
                        for region in self.robotLocations.keys():
                            if self.robotLocations[region][item.group("robotName")] and ((region.rfind('_rc') > 0) == RCregion):
                                self.robotLocations[region][item.group("robotName")] = False

                            if item.group("packageValue") == region:
                                self.robotLocations[region][item.group("robotName")] = True

                        printRegionInfo(self.robotLocations)

                    elif item.group('packageType') in ['SysInit', 'SysTrans', 'SysGoals', 'EnvInit', 'EnvTrans', 'EnvGoals']:
                        if ast.literal_eval(item.group("packageValue")):
                            # We got spec from robotClient, save spec
                            self.spec[item.group('packageType')][item.group("robotName")] = ast.literal_eval(item.group("packageValue"))
                            printSpec(item.group('packageType'), self.spec[item.group('packageType')][item.group("robotName")], item.group("robotName"))

                    elif item.group('packageType') == 'SysGoalsOld':
                        if ast.literal_eval(item.group("packageValue")):
                            # We got spec from robotClient, save spec
                            self.sysGoalsOld[item.group("robotName")] = ast.literal_eval(item.group("packageValue"))
                            printSpec(item.group('packageType'), self.sysGoalsOld[item.group("robotName")], item.group("robotName"))

                    elif item.group('packageType') == 'WinPos':
                        if ast.literal_eval(item.group("packageValue")):
                            # We got spec from robotClient, save spec
                            self.winPos[item.group("robotName")] = ast.literal_eval(item.group("packageValue"))
                            printSpec(item.group('packageType'), self.winPos[item.group("robotName")], item.group("robotName"))


                    elif item.group('packageType') == "coordinationRequest":
                        # receiving coorindation request
                        if isinstance(ast.literal_eval(item.group("packageValue")), bool):
                            self.coordinationRequest[item.group("robotName")] = ast.literal_eval(item.group("packageValue"))

                    elif item.group('packageType') == "centralizedExecutionStatus":
                        if self.centralizedExecutionStatus: # centralized mode. The other robot is restarting.. we should be soon?
                            self.tempRobotStatusOnCentralizedStrategyExecution[item.group("robotName")] = ast.literal_eval(item.group("packageValue"))
                        else:
                            # receive the centralized execution status of the other robots
                            self.robotStatusOnCentralizedStrategyExecution[item.group("robotName")] = ast.literal_eval(item.group("packageValue"))

                    elif item.group('packageType') in ['envPropList', 'sysPropList']:
                        # store propositions list
                        if item.group('packageType') == 'envPropList':
                            self.envPropList[item.group("robotName")] = ast.literal_eval(item.group("packageValue"))
                        else:
                            self.sysPropList[item.group("robotName")] = ast.literal_eval(item.group("packageValue"))

                        # save proposition mapping
                        for prop in ast.literal_eval(item.group("packageValue")).keys():
                            self.propMappingNewToOld[item.group("robotName")].update({prop:prop})
                            self.propMappingOldToNew[item.group("robotName")].update({prop:prop})

                    elif item.group('packageType') in 'nextPossibleStates':
                        if ast.literal_eval(item.group("packageValue")):
                            self.nextPossibleStatesArray[item.group("robotName")] = ast.literal_eval(item.group("packageValue"))

                    elif item.group('packageType') in 'restartStatus':
                        if ast.literal_eval(item.group("packageValue")):
                            self.readyToRestart[item.group("robotName")] = ast.literal_eval(item.group("packageValue"))

                    elif "closeConnection" in data:
                        x.close()
                        # first find key in clients and then delete it
                        del self.clients[self.robotAddresses.keys()[self.robotAddresses.values().index(x.getsockname())]]
                        logging.info('PATCHING_EXECUTOR: client ' + str(x) + 'is removed.')

                        # stop the checkData thread from running
                        self.keepConnection = False

                    else:
                        pass

        # Handle outputs
        for x in outputReady:
            queueEmpty = False
            while not queueEmpty:
                try:
                    next_msg = self.message_queues[x].get_nowait()
                except Queue.Empty:
                    # No messages waiting so stop checking for writability.
                    queueEmpty = True
                    #logging.warning('output queue for' + str(x.getpeername()) +  'is empty')
                else:
                    #logging.info('sending {0} to {1}'.format(repr(next_msg), x.getpeername()))
                    x.send(next_msg)

        # Handle "exceptional conditions"
        for x in exceptReady:
            logging.warning('handling exceptional condition for' + str(x.getpeername()))
            # Stop listening for input on the connection
            del self.clients[self.robotAddresses.keys()[self.robotAddresses.values().index(x.getsockname())]]
            x.close()
            del self.message_queues[x] # Remove message queue

    def runCheckData(self):
        """
        This function is created to run checkData with threading
        """
        while self.keepConnection:
            self.checkData()

            # check if need to drag neighbour robots in
            self.updateCoordinatingRobots(self.otherRobotsWithSelf, [])

    def runIterationNotCentralExecution(self):
        """
        This function runs one iteration of the execution when no centralized strategy is executing
        when self.centralizedExecutionStatus is False or None
        """
        #self.checkData()

        # check if need to drag neighbour robots in
        #self.updateCoordinatingRobots(self.otherRobotsWithSelf, [])

        if self.readyToRestart and (not False in self.readyToRestart.values()):
            logging.debug("We are now cleaning variables.")
            #clean all necessary variables when done
            self.checkedRestartStatus = True
            self.cleanVariables(first_time=False)

    def prepareForCentralizedExecution(self):
        """
        This function prepares for centralized execution.
        Returns True if centralized execution is ready and false otherwise.
        """
        # start patching (check if we get all the necessary spec and props)
        if self.coordinationRequest.values().count(True) and self.checkIfNecessaryPartsObtained():

            logging.info('We will start patching')

            # first update centralized execution status
            self.centralizedExecutionStatus = False

            # reconstruct new centralized spec
            self.renameSpecAndStoreNewMapping()

            # make sure next sysProp is not in EnvTrans and sysProp is not in EnvInit
            self.reconstructSpec()

            # synthesize new controller
            self.compileCentralizedSpec()

            # now we will start centralized execution
            self.centralizedExecutionStatus = True

            # notify the other robot that we are done with synthesis
            for robot in self.coordinatingRobots:
                if robot != self.robotName:
                    self.sendCentralizedExecutionStatus(self.clients[robot])
            return True

        else:
            return False

    def runIterationCentralExecution(self):
        """
        This function runs one iteration of centralized execution
        when self.centralizedExecutionStatus is True
        """
        #logging.debug('Now executing the centralized strategy...')
        #self.checkData()

        # check if need to drag neighbour robots in
        #self.updateCoordinatingRobots(self.otherRobotsWithSelf, [])

        # set time to check if sysGoals is satisfied
        # self.toc = time.time()
        # if (self.toc-self.tic) > 1:
        #     self.tic = time.time()
        # check if goals are satisfied
        if self.goalsSatisfied: #checkIfGoalsAreSatisfied():
            logging.debug('The centralized system goal is satisfied.')
            # reset status
            self.centralizedExecutionStatus = None
            # TODO: what if we have two instances of patching in parallel? Can't deal with it now.
            self.coordinationRequest = {k:False for k in self.coordinationRequest.keys()}
            #self.keepConnection = False

            # To do: make sure both robots are ready first, before we go back to local execution.
            for robot in list(set(self.coordinatingRobots) - set([self.robotName])):
                self.readyToRestart[robot] = False
                self.checkedRestartStatus = False

    # def run(self):
    #     """
    #     main loop of the centralExecutor object.
    #     """
    #     signal.signal(signal.SIGINT, self.closeConnection) # terminate object if keyboard interrupt

    #     while self.keepConnection:
    #         self.checkData()

    #         # check if need to drag neighbour robots in
    #         self.updateCoordinatingRobots(self.otherRobotsWithSelf, [])

    #         # start patching (check if we get all the necessary spec and props)
    #         if self.coordinationRequest.values().count(True) and self.checkIfNecessaryPartsObtained():

    #             logging.info('We will start patching')

    #             # first update centralized execution status
    #             self.centralizedExecutionStatus = False

    #             # reconstruct new centralized spec
    #             self.renameSpecAndStoreNewMapping()

    #             # make sure next sysProp is not in EnvTrans and sysProp is not in EnvInit
    #             self.reconstructSpec()

    #             # synthesize new controller
    #             self.compileCentralizedSpec()

    #             # now we will start centralized execution
    #             self.centralizedExecutionStatus = True

    #             # TODO: here run the centralized aut. also need to checkData
    #             tic = time.time()
    #             while self.centralizedExecutionStatus is not None:
    #                 #logging.debug('Now executing the centralized strategy...')
    #                 self.checkData()

    #                 #TODO: to remove for real execution
    #                 #self.testTriggerSysGoalsSatisfaction()

    #                 # set time to check if sysGoals is satisfied
    #                 toc = time.time()
    #                 if (toc-tic) > 5:
    #                     tic = time.time()
    #                     # check if goals are satisfied
    #                     if self.checkIfGoalsAreSatisfied():
    #                         logging.debug('The centralized system goal is satisfied.')
    #                         # reset status
    #                         self.centralizedExecutionStatus = None
    #                         # TODO: what if we have two instances of patching in parallel? Can't deal with it now.
    #                         self.coordinationRequest = {k:False for k in self.coordinationRequest.keys()}
    #                         #self.keepConnection = False

    #                         # To do: make sure both robots are ready first, before we go back to local execution.
    #                         for robot in self.coordinatingRobots - [self.robotName]:
    #                             self.readyToRestart[robot] = False
    #                             self.checkedRestartStatus = False

    #             #self.closeConnection(None,None)
    #             pass

    #         if self.readyToRestart and (not False in self.readyToRestart.values()):
    #             logging.debug("We are now cleaning variables.")
    #             #clean all necessary variables when done
    #             self.checkedRestartStatus = True
    #             self.cleanVariables(first_time=False)
    #     else:
    #         self.closeConnection(None, None)

    def closeConnection(self, signal, frame):
        """
        This function terminates all the communication with the clients.
        """
        for x in self.clients.values():
            x.close()
        self.serv.close()

        logging.info("PATCHING_EXECUTOR: Closing Connection.")
        #sys.exit()


    def checkIfNecessaryPartsObtained(self):
        """
        Obtain spec from each robot.
        """
        # check if all necessary specs are obtained. If that's not the case return and recevie info
        for robot in self.coordinatingRobots:
            for specType in self.spec.keys():
                if not robot in self.spec[specType]:
                    logging.warning('We have not recevied specs from ' + str(robot))
                    time.sleep(1)
                    return False

            if not (robot in self.sysPropList and robot in self.envPropList):
                logging.warning('We have not recevied all propositions from ' + str(robot))
                time.sleep(1)
                return False

        # receive all props + values
        logging.info('We got all the parts!')
        time.sleep(1)
        return True

    def renameSpecWithAllRobots(self, spec):
        """
        This function takes in a spec dict with keys to be robots and rename propositions in it.
        Helper Fn of renameSpecAndStoreNewMapping
        we are modifying the reference one so no return is needed?
        """
        # e.region to s.region (region heading)
        for robot in self.coordinatingRobots:
            for reg in self.robotLocations.keys():
                for otherRobot in self.coordinatingRobots:
                    spec[robot] = re.sub('(?<=[! &|(\t\n])'+'e.'+otherRobot+'_'+reg+'(?=[ &|)\t\n])', 's.'+otherRobot+'_'+reg, spec[robot])

        # append robot name in front of actuators and sensors props, but not regions
        ## sensor props
        ## e.g: sensorProp  -> robotName_sensorProp
        for robot, ePropList in self.envPropList.iteritems():
            for eProp, eValue in ePropList.iteritems():
                #ignore any region related props
                if eProp in [otherRobot+'_'+reg+'_rc' for reg in self.robotLocations.keys() for otherRobot in self.coordinatingRobots] or eProp in [otherRobot+'_'+reg for reg in self.robotLocations.keys() for otherRobot in self.coordinatingRobots if otherRobot != robot]:
                    continue
                spec[robot] = re.sub('(?<=[! &|(\t\n])'+'e.'+eProp+'(?=[ &|)\t\n])', 'e.'+robot+'_'+eProp, spec[robot])


        ## actuator props
        ## e.g: actuatorProp  -> robotName_actuatorProp
        for robot, sPropList in self.sysPropList.iteritems():
            for sProp, sValue in sPropList.iteritems():
                #ignore any region related props
                if sProp in [otherRobot+'_'+reg for reg in self.robotLocations.keys() for otherRobot in self.coordinatingRobots]:
                    continue
                spec[robot] = re.sub('(?<=[! &|(\t\n])'+'s.'+sProp+'(?=[ &|)\t\n])', 's.'+robot+'_'+sProp, spec[robot])

    def renameSpecAndStoreNewMapping(self):
        """
        Rename specs and store mapping.
        """

        """
        Rename specs
        """
        for specType in self.spec.keys():
            self.renameSpecWithAllRobots(self.spec[specType])

        # also just for sysGoalsOld
        self.renameSpecWithAllRobots(self.sysGoalsOld)

        # also for winPos
        self.renameSpecWithAllRobots(self.winPos)

        """
        Store mapping
        """
        states = strategy.StateCollection()

        logging.debug("self.old_coordinatingRobots:" + str(self.old_coordinatingRobots))

        # store env props
        for robot, ePropList in self.envPropList.iteritems():
            # check if those robots are in centralized mode already.
            if not robot in self.old_coordinatingRobots: # not in centralized mode
                for eProp, eValue in ePropList.iteritems():
                    if eProp in [robot+'_'+reg+'_rc' for reg in self.robotLocations.keys()]:
                        # keep original name
                        self.smvEnvPropList.append(eProp)
                        self.currentAssignment.update({eProp: eValue})

                    # e.g: remove alice_r4 in bob's sensors. However, for robot not coorindating, we might have alice_charlie_r1 and bob_charlie_r1 in our smv file
                    elif eProp in [otherRobot+'_'+reg for reg in self.robotLocations.keys() for otherRobot in self.coordinatingRobots if otherRobot != robot]:
                        # store and update prop mapping
                        self.propMappingNewToOld[robot].pop(eProp)
                        self.propMappingNewToOld[robot][eProp+'_rc'] = eProp
                        self.propMappingOldToNew[robot][eProp] = eProp+'_rc'
                        #continue

                    # the similar case of _rc
                    elif eProp in [otherRobot+'_'+reg+'_rc' for reg in self.robotLocations.keys() for otherRobot in self.coordinatingRobots if otherRobot != robot]:
                        continue

                    else:
                        # store and update prop mapping
                        self.propMappingNewToOld[robot].pop(eProp)
                        self.propMappingNewToOld[robot][robot+'_'+eProp] = eProp
                        self.propMappingOldToNew[robot][eProp] = robot+'_'+eProp
                        self.smvEnvPropList.append(robot+'_'+eProp)
                        self.currentAssignment.update({robot+'_'+eProp: eValue})

            else: # robot in centralized mode.
                # update assignments only
                self.currentAssignment.update({k:v for k, v in self.sensor_state.getInputs(expand_domains=True).iteritems() if robot in k})

        # add input props to states collection
        states.addInputPropositions(self.smvEnvPropList)

        # stroe sys props
        for robot, sPropList in self.sysPropList.iteritems():
            # check if those robots are in centralized mode already.
            if not robot in self.old_coordinatingRobots: # not in centralized mode
                for sProp, sValue in sPropList.iteritems():
                    if sProp in [robot+'_'+reg for reg in self.robotLocations.keys()]:
                        # keep original name
                        self.smvSysPropList.append(sProp)
                        self.currentAssignment.update({sProp: sValue})
                    else:
                        # store and update prop mapping
                        self.propMappingNewToOld[robot].pop(sProp)
                        self.propMappingNewToOld[robot][robot+'_'+sProp] = sProp
                        self.propMappingOldToNew[robot][sProp] = robot+'_'+sProp
                        self.smvSysPropList.append(robot+'_'+sProp)
                        self.currentAssignment.update({robot+'_'+sProp: sValue})
            else: # robot in centralized mode.
                # update assignments only. make reg_rc and reg the same
                sysProps = {k:v for k, v in self.strategy.current_state.getOutputs(expand_domains=True).iteritems() if robot in k}
                for eProp, eValue in {k:v for k, v in self.sensor_state.getInputs(expand_domains=True).iteritems() if robot in k}.iteritems():
                    for reg in self.robotLocations.keys():
                        if reg in eProp:
                            sysProps[eProp.replace('_rc', '')] = eValue
                self.currentAssignment.update(sysProps)

        # add input props to states collection
        states.addOutputPropositions(self.smvSysPropList)

        # store current state
        self.currentState = states.addNewState(self.currentAssignment)

    def reconstructSpec(self):
        """
        Move clauses with next sysProp from EnvTrans to SysTrans.
        """
        for robot, spec in self.spec['EnvTrans'].iteritems():
            newEnvTrans, extraSysTrans = LTLParser.LTLcheck.separateLTLwithNextSystemProps(spec)
            self.spec['EnvTrans'][robot] = newEnvTrans
            self.spec['SysTrans'][robot] = '&\n'.join(filter(None, LTLParser.LTLcheck.ltlStrToList(self.spec['SysTrans'][robot])+[extraSysTrans]))

        for robot, spec in self.spec['EnvInit'].iteritems():
            newEnvInit, extraSysInit = LTLParser.LTLcheck.separateLTLwithoutEnvPropFromEnvInit(spec)
            self.spec['EnvInit'][robot] = newEnvInit
            self.spec['SysInit'][robot] = '&\n'.join(filter(None, [self.spec['SysInit'][robot], extraSysInit]))


    def compileCentralizedSpec(self):
        """
        Compile centralized spec.
        """
        # synthesize our new centralized controller again
        self.compiler = specCompiler.SpecCompiler()
        self.compiler.proj.compile_options['synthesizer'] = 'slugs' # use slugs
        self.compiler.proj.project_root = os.path.dirname(os.path.realpath(__file__)) #set directory to save slugsin
        self.compiler.proj.project_basename = self.filePath
        createSMVfile(self.filePath, self.smvEnvPropList, self.smvSysPropList) # create a new SMV file

        # create a new LTL file
        LTLspec_envList = []
        LTLspec_sysList = []

        # append new sys and env init
        currentEnvInitState = '('+self.currentState.getLTLRepresentation(mark_players=True, use_next=False, include_inputs=True, include_outputs=False)+')'
        currentSysInitState = '('+self.currentState.getLTLRepresentation(mark_players=True, use_next=False, include_inputs=False, include_outputs=True)+')'

        ## construct specification
        for specType in self.spec.keys():
            if specType in ['EnvInit', 'SysInit']:
                if specType == 'EnvInit': # append init state
                    LTLspec_envList.append(currentEnvInitState)
                else:
                    LTLspec_sysList.append(currentSysInitState)

            for robot in self.coordinatingRobots:
                # here we will skip the goals and join them together below
                if specType in ['SysGoals']:
                    continue
                elif specType in ['SysInit', 'SysTrans']:
                    LTLspec_sysList.append(self.spec[specType][robot])
                else:
                    LTLspec_envList.append(self.spec[specType][robot])

        # join the goals of the robots so that the goals are pursued at the same time
        # ----- OLD ------- #
        #specSysGoals = " &\n ".join(filter(None, [x.strip().lstrip('[]<>') for x in self.spec['SysGoals'].values()]))
        #LTLspec_sysList.append("[]<>(" + specSysGoals + ")" if specSysGoals else specSysGoals)
        # ----------------- #

        # ---- []<>(goal1 and winPos) & []<>(goal2 and winPos) ----- #
        # sysGoalsList = []
        # for x in self.spec['SysGoals'].values():
        #     sysGoalsList.append("[]<>(" + " &\n ".join(filter(None, [x.strip().lstrip('[]<>')] + self.winPos.values())) + ")")

        # specSysGoals = " &\n ".join(filter(None, sysGoalsList))
        # logging.debug("specSysGoals:" + str(specSysGoals))
        # LTLspec_sysList.append(specSysGoals)

        # ---- []<>(goal1) & []<>(goal2) & []<>(winPos) ---- #
        LTLspec_sysList.append(" &\n ".join(filter(None, self.spec['SysGoals'].values() + ["[]<>(" + " &\n ".join(filter(None, self.winPos.values())) + ")"])))


        # set up violation check object
        # ------ OLD ------- #
        # specSysGoalsOld = " &\n ".join(filter(None, [x.strip().lstrip('[]<>') for x in self.sysGoalsOld.values()]))
        # logging.debug("specSysGoalsOld:" + str(specSysGoalsOld))
        # if specSysGoalsOld:
        #     self.sysGoalsCheck = LTLParser.LTLcheck.LTL_Check(None, {}, {'sysGoals':specSysGoalsOld}, 'sysGoals')
        # ------ check winning positions and each goal ------ #
        for robot in self.spec['SysGoals'].keys():
            self.sysGoalsCheck[robot] = LTLParser.LTLcheck.LTL_Check(None, {}, {'SysGoals':self.spec['SysGoals'][robot]}, 'SysGoals')
            self.sysGoalsCheckStatus[robot] = False
        specWinPos = "[]<>(" + " &\n ".join(filter(None, self.winPos.values())) + ")"
        if specWinPos:
            self.winPosCheck = LTLParser.LTLcheck.LTL_Check(None, {}, {'WinPos':specWinPos}, 'WinPos')

        createLTLfile(self.filePath, " &\n".join(filter(None, LTLspec_envList)), " &\n".join(filter(None, LTLspec_sysList)))
        startTime = time.time()
        #HACK: Make it to recovery mode to try it out
        #self.compiler.proj.compile_options['recovery']=True # interactive strategy auto synthesizes with recovery option
        self.compiler.proj.compile_options["cooperative_gr1"] = True
        self.compiler.proj.compile_options["symbolic"] = False
        #self.compiler.proj.compile_options["interactive"] = True
        #self.compiler.proj.compile_options["only_realizability"] = True
        realizable, realizableFS, output = self.compiler._synthesize()
        endTime = time.time()
        logging.info(output)

        """
        If realizable, load AUT and return status to each robot. The execution of each robot resumes.
        """
        if realizable:
            logging.info('Strategy synthesized in ' + str(endTime-startTime)+' s.')
            # load strategy and initial state
            if not self.compiler.proj.compile_options["symbolic"]: # explicit strategy
                self.strategy = strategy.createStrategyFromFile(self.filePath + '.aut', self.smvEnvPropList, self.smvSysPropList)
            elif self.compiler.proj.compile_options["interactive"]:
                self.strategy = strategy.createStrategyFromFile(self.filePath + '.slugsin', self.smvEnvPropList, self.smvSysPropList)
            else:
                logging.warning("Please note that bdd is not tested yet.")
                self.strategy = strategy.createStrategyFromFile(self.filePath + '.bdd', self.smvEnvPropList, self.smvSysPropList)

            # search for init state.
            self.strategy.current_state = self.strategy.searchForOneState(self.currentAssignment)
            logging.info('Starting at State ' + str(self.strategy.current_state.state_id))
        else:
            logging.error('cannot synthesize a centralized patch')
            self.closeConnection(None, None)
            sys.exit()

        # set up sensor_strategy for runtime monitoring the liveness condition
        self.sensor_state = self.strategy.states.addNewState()
        self.sensor_state.setPropValues(self.currentState.getInputs(expand_domains=True))

    def findRobotOutputs(self, robot, nextInputs):
        """
        receive latest inputs from one robot and find the next outputs of the robot.
        Inputs:
        robot: name of the robot
        nextInputs: dictionary of inputs with original prop name from the robot

        Outputs:
        nextOutputs: dictionary of outputs with original prop name from the robot
        """

        # convert to the new mapping name from received data / update current assignments
        currentInputs = self.strategy.current_state.getInputs(expand_domains=True)

        # convert local props to global props
        nextInputs = {self.propMappingOldToNew[robot][k]:v for k, v in nextInputs.iteritems()}

        # remove any envProps that are sysProps in nextInputs
        nextInputs = ({k:v for k, v in nextInputs.iteritems() if k in currentInputs.keys()})

        # update inputs based on the inputs from the other robot
        currentInputs.update(nextInputs)

        # update sensor_state with the latest information
        #self.sensor_state.setPropValues(nextInputs)

        # find if the state is changed
        next_states = self.strategy.findTransitionableStates(currentInputs, from_state=self.strategy.current_state)

        ## This is from executeStrategy.py
        # Make sure we have somewhere to go
        if len(next_states) == 0:
            # Well darn!
            logging.error("Could not find a suitable state to transition to!")
            logging.debug("nextInputs:" + str([k for k, v in nextInputs.iteritems() if v]))
            logging.debug("currentInputs:" + str([k for k, v in self.strategy.current_state.getInputs(expand_domains=True).iteritems() if v]))
            return

        # See if we're beginning a new transition
        if [next_state.getAll(expand_domains=True) for next_state in next_states] != [last_next_state.getAll(expand_domains=True) for last_next_state in self.last_next_states]:
            # NOTE: The last_next_states comparison is also to make sure we don't
            # choose a different random next-state each time, in the case of multiple choices
            self.last_next_states = next_states

            # Only allow self-transitions if that is the only option!
            if len(next_states) > 1 and self.strategy.current_state in next_states:
                next_states.remove(self.strategy.current_state)

            next_state = random.choice(next_states)

            if next_state.getAll(expand_domains=True) != self.strategy.current_state.getAll(expand_domains=True):
                self.strategy.current_state = next_state
                self.last_next_states = []  # reset
                logging.info('Currently at State ' + str(self.strategy.current_state.state_id))
                logging.debug(str([prop for prop, value in self.strategy.current_state.getAll(expand_domains=True).iteritems() if value]))
                logging.debug('---------------------------------------------------------')

            # check if state is satisified
            if not self.checkSysGoalsThread or not self.checkSysGoalsThread.isAlive():
                self.runSingleTime_checkIfGoalsAreSatisfied() #self.checkIfGoalsAreSatisfied()

        """
        send updated outputs to the robot
        """
        # convert to original prop names and send back to the robot
        nextOutputs = {}
        for prop in self.sysPropList[robot]:
            nextOutputs[prop] = self.strategy.current_state.getAll(expand_domains=True)[self.propMappingOldToNew[robot][prop]]

        return nextOutputs

    def runSingleTime_checkIfGoalsAreSatisfied(self):
        """
        This function runs checkIfGoalsAreSatisfied once in thread.
        """
        self.checkSysGoalsThread = threading.Thread(target=self.checkIfGoalsAreSatisfied, args=())
        self.checkSysGoalsThread.start()

    def startCheckSysGoalsThread(self):
        """
        this functions starts the thread to run run_checkIfGoalsAreSatisfied.
        """
        self.checkSysGoalsThread = threading.Thread(target=self.run_checkIfGoalsAreSatisfied, args=())
        self.checkSysGoalsThread.daemon = True  # Daemonize thread
        self.checkSysGoalsThread.start()

    def run_checkIfGoalsAreSatisfied(self):
        """
        thread version of checkIfGoalsAreSatisfied.
        """
        old_current_state = None
        while not self.goalsSatisfied:
            # only carry out the check when state is updated
            if old_current_state is None or old_current_state.getAll(expand_domains=True) != self.strategy.current_state.getAll(expand_domains=True):
                self.goalsSatisfied = self.checkIfGoalsAreSatisfied()
            old_current_state = self.strategy.current_state

    def checkIfGoalsAreSatisfied(self):
        """
        Constant check if goal is achieved. If so, terminate cooridation.
        Return true if goals are satisfied and false otherwise.
        """
        winPosStatus = False
        startTime = time.time()
        if False in self.sysGoalsCheckStatus.values():
            for robot in self.sysGoalsCheck.keys():
                if not self.sysGoalsCheckStatus[robot]:
                    self.sysGoalsCheckStatus[robot] = self.sysGoalsCheck[robot].checkViolation(self.strategy.current_state, self.strategy.current_state)
                    logging.debug("Is sysGoals of " + robot + " satisfied? " + str(self.sysGoalsCheckStatus[robot]))
                    logging.debug("System goal:" + str(self.sysGoalsCheck[robot].env_safety_assumptions))
                    logging.debug("current_state:" + str([k for k, v in self.strategy.current_state.getAll(expand_domains=True).iteritems() if v]))

        if not False in self.sysGoalsCheckStatus.values(): # now we can check winning positions
            winPosStatus = self.winPosCheck.checkViolation(self.strategy.current_state, self.strategy.current_state)
            logging.debug("Are we in winning positions?:" + str(winPosStatus))
            logging.debug("current_state: no-" + str(self.strategy.current_state.state_id) + "," + str([k for k, v in self.strategy.current_state.getAll(expand_domains=True).iteritems() if v]))

        logging.debug("time taken:" + str(time.time() - startTime))

        self.goalsSatisfied = (False if False in self.sysGoalsCheckStatus.values() else winPosStatus)
        return (False if False in self.sysGoalsCheckStatus.values() else winPosStatus)

    def testTriggerSysGoalsSatisfaction(self):
        """
        testing function only: to trigger goal reaching
        """
        # try to trigger goal satisfaction
        self.strategy.current_state = random.choice([x for x in self.strategy.searchForStates({'bob_r3_rc':0, 'bob_r1_rc':0, 'bob_r2_rc':0, 'bob_r5_rc':1, 'bob_r4_rc':0, 'alice_r5_rc':0, 'alice_r1_rc':1, 'alice_r2_rc':0, 'alice_r3_rc':0, 'alice_r4_rc':0, 'bob_r4':0, 'bob_r5':0, 'bob_r1':0, 'bob_r2':0, 'bob_r3':1, 'alice_r5':0, 'alice_r4':0, 'alice_r1':0, 'alice_r3':0, 'alice_r2':1})])

    def updateCoordinatingRobots(self, functionName, functionPara):
        """
        This function checked if coorindation request is received.
        functionName: name of the function (with no parenteses)
        functionPara: parameters of the function

        Update:
        self.coordinatingRobots object.
        """

        oldCoordinatingRobots = copy.deepcopy(self.coordinatingRobots)
        self.coordinatingRobots = functionName(functionPara)

        # coorindating robots has changed. everything should be redone.
        if oldCoordinatingRobots != self.coordinatingRobots:
            self.old_coordinatingRobots = oldCoordinatingRobots
            self.resetRobotStatusOnCentralizedStrategyExecution()

        #TODO: need to track if coorindating robots has changed.

    def resetRobotStatusOnCentralizedStrategyExecution(self):
        """
        This function resets self.robotStatusOnCentralizedStrategyExecution based on coordinatingRobots.
        (In the case where global EnvTrans is violated and spec is resynthesized as well.)
        """
        for robot in self.coordinatingRobots:
            if robot != self.robotName:
                self.robotStatusOnCentralizedStrategyExecution[robot] = False

    def otherRobotsWithSelf(self, para):
        """
        * To be passed to updateCoordinatingRobots
        If any robot toggled patching status to true, this function change that robot's patching receival status to true.
        Also include ourself in the list.
        para: parameters. Not used here.

        Output:
        {robot:True} dict if any robot's pathcing status is true and {robot:False} otherwise
        """
        if self.coordinationRequest.values().count(True) or self.coordinationRequestSent:
            # also make sure there are no duplicates
            #logging.debug('robots coordinating:' + str(list(set([robot for robot, status in self.coordinationRequest.iteritems() if status] + [self.robotName] + self.coordinationRequestSent))))
            return list(set([robot for robot, status in self.coordinationRequest.iteritems() if status] + [self.robotName] + self.coordinationRequestSent))
        else:
            return []

    def setCoordinationRequestSent(self, robotList):
        """
        This function updates self.coordinationRequestSent
        """
        self.coordinationRequestSent = copy.deepcopy(robotList)


if __name__ == "__main__":
    logging.info('Please run testDPatching in helperFns to test the module.')
