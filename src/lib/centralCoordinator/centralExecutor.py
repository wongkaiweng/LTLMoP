#centralExecutor.py
import socket #import the socket library
import logging
import select #for multiple robot client connections
import re     #for parsing msg from client
import ast    #for parsing msg from client
import sys    #for program exiting
import signal #for terminating object when keyboard interrupt
import time   #for temporary pause, timing synthesis time
import random #for chosing one next states from the list

# Climb the tree to find out where we are
import os, sys
p = os.path.abspath(__file__)
t = ""
while t != "src":
    (p, t) = os.path.split(p)
    if p == "":
        print "I have no idea where I am; this is ridiculous"
        sys.exit(1)

sys.path.append(os.path.join(p,"src","lib"))

#for compiling the centralized spec
import specCompiler
from createJTLVinput import createLTLfile, createSMVfile
import strategy
import LTLParser.LTLcheck

logging.getLogger().setLevel(logging.DEBUG)
#logging.basicConfig(level=logging.DEBUG)
#logger = logging.getLogger(__name__)

##let's set up some constants
HOST = ''    #we are the host
ADDR = ("localhost",6501)    #we need a tuple for the address
BUFSIZE = 20000    #reasonably sized buffer for data

def printRegionInfo(regionList):
    """
    This function prints the current region information of each robot
    """
    logging.info('----------------------------------------------')
    for region, robots in regionList.iteritems():
        table = ""
        for rob, status in robots.iteritems():
            table = table + "-{0:10}: {1:6} ".format(rob, status)
        table = "{0:13}".format(region) + table
        logging.info(table)

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


class CentralExecutor:

    """
    LTLMoP robot object used to communicate with the negotiationMonitor
    """
    def __init__(self):

        self.cleanVariables() #initialize all necessary variables

        self.strategy = None # for the loaded new centralized strategy

        self.filePath = 'centralizedSpec' #output SMV, LTL file directory and name

        ## now we create a new socket object (serv)
        self.serv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        listenConn = 5  #listenConn is the maximum number of queued connections we'll allow

        #find our socket to the address
        self.serv.bind((ADDR))    #the double parens are to create a tuple with one element
        self.serv.listen(listenConn)
        logging.info( 'CENTRAL_EXECUTOR: Listening...')

        self.keepConnection = True
        self.clients = [self.serv]


    def checkData(self):
        """
        Check if data is received and update info.
        """
        inputReady, outputReady, exceptReady = select.select(self.clients, [], [], 0)
        for x in inputReady:

            if x == self.serv:
                csock, addr = self.serv.accept()
                self.clients.append(csock)
                #logging.info('CENTRAL_EXECUTOR: ...connected! client list = ' + str(clients))

                # for tempMsg
                self.tempMsg[csock] = ""

            else:
                # find data pattern
                data = x.recv(BUFSIZE)

                # retrieve incomplete data if there's any
                data = self.tempMsg[x]+data
                self.tempMsg[x] = ""

                # parse msg
                pattern ="(?P<robotName>\w+)-(?P<packageType>\w+)\s=\s(?P<packageValue>\[?.+\]?)\n"      # MSG FORMAT
                result = re.finditer(pattern, data)

                # temporarily save incomplete msg
                if not re.match(pattern, data):
                    self.tempMsg[x] = data
                else: # we have some patterns matching but the last one might be incomplete
                    self.tempMsg[x] = data[data.rfind('\n')+1:]

                for item in result:
                    if item.group('packageType')  == "regionList":
                        # save region info and parse list into a dict of the form  regionList[r1][robot] = true

                        # convert region list string to list
                        rList = ast.literal_eval(item.group("packageValue"))

                        # store current status of the robot be all false
                        for reg in rList:
                            if not reg in self.regionList.keys():
                                self.regionList[reg] = {}
                            self.regionList[reg][item.group("robotName")] = False

                        printRegionInfo(self.regionList)

                        # append robotName to the list of coorindating robots
                        #self.coordinatingRobots.append(item.group("robotName"))

                        # set patching status to false
                        self.patchingStatus[item.group("robotName")] = False

                        # set recevial of patching request to false
                        self.patchingRequestReceived[item.group("robotName")] = False

                        # set up spec for the robot
                        for specType, value in self.spec.iteritems():
                            self.spec[specType][item.group("robotName")] = ""

                        # set up propMappingNewToOld for the robot
                            self.propMappingNewToOld[item.group("robotName")] = {}
                            self.propMappingOldToNew[item.group("robotName")] = {}

                    elif item.group('packageType')  ==  "regionName":
                        # first figure out if it's rc region or not
                        RCregion = False
                        if item.group("packageValue").rfind('_rc') > 0:
                            RCregion = True

                        # update region info of robot
                        for region, robots in self.regionList.iteritems():
                            if self.regionList[region][item.group("robotName")] and ((region.rfind('_rc')>0) == RCregion):
                                self.regionList[region][item.group("robotName")] = False

                            if item.group("packageValue") == region:
                                self.regionList[region][item.group("robotName")] =True

                        printRegionInfo(self.regionList)

                    elif item.group('packageType') in ['SysInit','SysTrans','SysGoals','EnvInit','EnvTrans','EnvGoals']:
                        if ast.literal_eval(item.group("packageValue")):
                            # We got spec from robotClient, save spec
                            self.spec[item.group('packageType')][item.group("robotName")] = ast.literal_eval(item.group("packageValue"))
                            printSpec(item.group('packageType'), self.spec[item.group('packageType')][item.group("robotName")], item.group("robotName"))
                        """
                        else:
                            # robotClient is requesting spec, send back spec
                            logging.info(item.group('packageType') + ' requested by ' + item.group("robotName"))
                            # check if we need to request spec from the other robot
                            # TODO: this might not be necessary.
                            for robot, specStr in self.spec[item.group('packageType')].iteritems():
                                if robot != item.group("robotName") and self.spec[item.group('packageType')][robot] == "":
                                    self.requestSpecStatus[robot].append(item.group('packageType'))

                            # send spec back to the robot
                            x.send(str(self.spec[item.group('packageType')]))
                        """

                    elif item.group('packageType')  == "patchingStatus":
                        if isinstance(ast.literal_eval(item.group("packageValue")), bool):
                            self.patchingStatus[item.group("robotName")] = ast.literal_eval(item.group("packageValue"))
                        """
                        else:
                            # send the patching status of the robot
                            x.send(str(self.patchingStatus[item.group("robotName")]))
                        """

                    elif item.group('packageType')  == "patchingRequest":
                            # send the receival of patching request of the robot
                            x.send(str(self.patchingRequestReceived[item.group("robotName")]))

                    elif item.group('packageType')  == "centralizedExecutionStatus":
                            # send the centralized execution status to the robot
                            x.send(str(self.centralizedExecutionStatus))

                    elif item.group('packageType')  == "sensorUpdate":
                        # send the list of region info
                        x.send(str(self.regionList))

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
                        else:
                            x.send(str(self.nextPossibleStatesArray))

                    elif item.group('packageType') in 'automatonExecution':
                        # first receive inputs
                        nextInputs = ast.literal_eval(item.group("packageValue")) # dict
                        robot = item.group("robotName")

                        # check for outputs
                        nextOutputs = self.findRobotOutputs(robot, nextInputs)
                        x.send(str(nextOutputs))

                    elif "closeConnection" in data:
                        x.close()
                        self.clients.remove(x)
                        logging.info('CENTRAL_EXECUTOR: client ' + str(x) + 'is removed.' )

                    else:
                        pass

    def run(self):
        """
        main loop of the centralExecutor object.
        """
        signal.signal(signal.SIGINT, self.closeConnection) # terminate object if keyboard interrupt

        while self.keepConnection:
            self.checkData()

            # check if need to drag neighbour robots in
            self.updatePatchingRequest(self.allRobotCentralized,[])

            # start patching (make sure no of patchinStatus is the same as no of receivedPatchingRequest)
            if self.patchingStatus.values().count(True) >= 2 and self.patchingStatus.values().count(True) == self.patchingRequestReceived.values().count(True):

                logging.info('We will start patching')

                # first update centralized execution status
                self.centralizedExecutionStatus = False

                # check if we get all the necessary spec and props
                self.checkIfNecessaryPartsObtained()

                # reconstruct new centralized spec
                self.renameSpecAndStoreNewMapping()

                # make sure next sysProp is not in EnvTrans and sysProp is not in EnvInit
                self.reconstructSpec()

                # synthesize new controller
                self.compileCentralizedSpec()

                # now we will start centralized execution
                self.centralizedExecutionStatus = True

                # TODO: here run the centralized aut. also need to checkData
                while self.keepConnection:
                    #logging.debug('Now executing the centralized strategy...')
                    self.checkData()

                    #TODO: to remove for real execution
                    #self.testTriggerSysGoalsSatisfaction()

                    # check if goals are satisfied
                    if self.checkIfGoalsAreSatisfied():
                        logging.debug('The centralized system goal is satisfied.')
                        # reset status
                        self.centralizedExecutionStatus = None
                        # TODO: what if we have two instances of patching in parallel? Can't deal with it now.
                        self.patchingRequestReceived = {k:False for k in self.patchingRequestReceived.keys()}
                        self.patchingStatus = {k:False for k in self.patchingStatus.keys()}
                        #self.keepConnection = False

                #clean all necessary variables when done
                self.cleanVariables()

                #self.closeConnection(None,None)
                pass
        else:
            self.closeConnection(None, None)

    def cleanVariables(self):
        #This function clean and initialize all variables when patching is done/ when the instance is first created
        # TODO: we might not need to clean up all variables when patching ends (still need to exchange region info))
        self.regionList = {}  #tracking region info for each robot
        self.spec       = {'EnvInit':{},'EnvTrans':{},'EnvGoals':{},'SysInit':{},'SysTrans':{},'SysGoals':{}}
        self.coordinatingRobots = [] #track the robots cooridnating
        self.envPropList = {} #tracking env propositions of each robot envPropList[robot]= propList = {prop:value}
        self.sysPropList = {} #tracking sys propositions of each robot sysPropList[robot]= propList = {prop:value}
        self.propMappingNewToOld = {} #track mapping from newPropName back to oldPropName propMappingNewToOld[robot][newPropName] = oldPropName
        self.propMappingOldToNew = {} #track mapping from newPropName back to oldPropName propMappingOldToNew[robot][oldPropName] = newPropName
        self.smvSysPropList = [] #store newPropName for system props
        self.smvEnvPropList = [] #store newPropName for environment props
        self.currentState = None #state object. store the combined current state of all robots
        self.currentAssignment = {} # dict. store assignments of all props and all robots
        self.patchingStatus = {} #track if patching is initiated. True if started and false otherwise
        self.patchingRequestReceived = {} #track if we are initializing patching. False is not and True otherwise. This is checked by every robot constantly to make sure they enter the mode when necessary
        self.last_next_states = [] #track the last next states in our autonmaton execution
        self.sysGoalsCheck = None # runtime monitoring object to check if goals are reached
        self.nextPossibleStatesArray = {} # store array of next possible states dict of robots. To be request by the other robots

        self.centralizedExecutionStatus = None # track centralized execution. True for centralized execution. False for waiting to execute centralized strategy. None for no centralized execution/execution ended.

    def closeConnection(self, signal, frame):
        """
        This function terminates all the communication with the clients.
        """
        for x in self.clients:
            x.close()
        self.serv.close()

        logging.info("CENTRAL_EXECUTOR: Closing Connection.")
        sys.exit()


    def checkIfNecessaryPartsObtained(self):
        """
        Obtain spec from each robot.
        """
        # figure out which robots are coordinating
        self.coordinatingRobots = [robot for robot, status in self.patchingStatus.items() if status]

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

    def renameSpecAndStoreNewMapping(self):
        """
        Rename specs and store mapping.
        """

        """
        Rename specs
        """
        for specType in self.spec.keys():
            # e.region to s.region (region heading)
            for robot in self.coordinatingRobots:
                for reg in self.regionList.keys():
                    for otherRobot in self.coordinatingRobots:
                        self.spec[specType][robot] = re.sub('(?<=[! &|(\t\n])'+'e.'+otherRobot+'_'+reg+'(?=[ &|)\t\n])', 's.'+otherRobot+'_'+reg, self.spec[specType][robot])

            # append robot name in front of actuators and sensors props, but not regions
            ## sensor props
            ## e.g: sensorProp  -> robotName_sensorProp
            for robot, ePropList in self.envPropList.iteritems():
                for eProp, eValue in ePropList.iteritems():
                    #ignore any region related props
                    if eProp in [otherRobot+'_'+reg+'_rc' for reg in self.regionList.keys() for otherRobot in self.coordinatingRobots] or eProp in [otherRobot+'_'+reg for reg in self.regionList.keys() for otherRobot in self.coordinatingRobots if otherRobot != robot]:
                        continue
                    self.spec[specType][robot] = re.sub('(?<=[! &(])'+'e.'+eProp+'(?=[ &)])', 'e.'+robot+'_'+eProp, self.spec[specType][robot])


            ## actuator props
            ## e.g: actuatorProp  -> robotName_actuatorProp
            for robot, sPropList in self.sysPropList.iteritems():
                for sProp, sValue in sPropList.iteritems():
                    #ignore any region related props
                    if sProp in [otherRobot+'_'+reg for reg in self.regionList.keys() for otherRobot in self.coordinatingRobots]:
                        continue
                    self.spec[specType][robot] = re.sub('(?<=[! &(])'+'s.'+sProp+'(?=[ &)])', 's.'+robot+'_'+sProp, self.spec[specType][robot])

        """
        Store mapping
        """
        states = strategy.StateCollection()

        # store env props
        for robot, ePropList in self.envPropList.iteritems():
            for eProp, eValue in ePropList.iteritems():
                if eProp in [robot+'_'+reg+'_rc' for reg in self.regionList.keys()]:
                    # keep original name
                    self.smvEnvPropList.append(eProp)
                    self.currentAssignment.update({eProp: eValue})

                # e.g: remove alice_r4 in bob's sensors. However, for robot not coorindating, we might have alice_charlie_r1 and bob_charlie_r1 in our smv file
                elif eProp in [otherRobot+'_'+reg for reg in self.regionList.keys() for otherRobot in self.coordinatingRobots if otherRobot != robot]:
                    continue

                # the similar case of _rc
                elif eProp in [otherRobot+'_'+reg+'_rc' for reg in self.regionList.keys() for otherRobot in self.coordinatingRobots if otherRobot != robot]:
                    continue

                else:
                    # store and update prop mapping
                    self.propMappingNewToOld[robot].pop(eProp)
                    self.propMappingNewToOld[robot][robot+'_'+eProp] = eProp
                    self.propMappingOldToNew[robot][eProp] = robot+'_'+eProp
                    self.smvEnvPropList.append(robot+'_'+eProp)
                    self.currentAssignment.update({robot+'_'+eProp: eValue})

        # add input props to states collection
        states.addInputPropositions(self.smvEnvPropList)

        # stroe sys props
        for robot, sPropList in self.sysPropList.iteritems():
            for sProp, sValue in sPropList.iteritems():
                if sProp in [robot+'_'+reg for reg in self.regionList.keys()]:
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
            self.spec['SysTrans'][robot] = '&\n'.join(filter(None,LTLParser.LTLcheck.ltlStrToList(self.spec['SysTrans'][robot])+[extraSysTrans]))

        for robot, spec in self.spec['EnvInit'].iteritems():
            newEnvInit, extraSysInit = LTLParser.LTLcheck.separateLTLwithoutEnvPropFromEnvInit(spec)
            self.spec['EnvInit'][robot] = newEnvInit
            self.spec['SysInit'][robot] = '&\n'.join(filter(None,[self.spec['SysInit'][robot],extraSysInit]))


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
            if specType in ['EnvInit','SysInit']:
                if specType == 'EnvInit': # append init state
                    LTLspec_envList.append(currentEnvInitState)
                else:
                    LTLspec_sysList.append(currentSysInitState)

            for robot in self.coordinatingRobots:
                # here we will skip the goals and join them together below
                if specType in ['SysGoals']:
                    continue
                elif specType in ['SysInit','SysTrans']:
                    LTLspec_sysList.append(self.spec[specType][robot])
                else:
                    LTLspec_envList.append(self.spec[specType][robot])

        # join the goals of the robots so that the goals are pursued at the same time
        specSysGoals = " & ".join(filter(None,[x.strip().lstrip('[]<>') for x in self.spec['SysGoals'].values()]))
        LTLspec_sysList.append("[]<>(" + specSysGoals + ")" if specSysGoals else specSysGoals)

        # set up violation check object
        if specSysGoals:
            self.sysGoalsCheck = LTLParser.LTLcheck.LTL_Check(None,{}, {'sysGoals':specSysGoals}, 'sysGoals')

        createLTLfile(self.filePath, " &\n".join(filter(None, LTLspec_envList)), " &\n".join(filter(None,LTLspec_sysList)))
        startTime = time.time()
        realizable, realizableFS, output  = self.compiler._synthesize()
        endTime = time.time()
        logging.info(output)

        """
        If realizable, load AUT and return status to each robot. The execution of each robot resumes.
        """
        if realizable:
            logging.info('Strategy synthesized in ' + str(endTime-startTime)+' s.')
            # load strategy and initial state
            self.strategy = strategy.createStrategyFromFile(self.filePath + '.aut', self.smvEnvPropList, self.smvSysPropList)
            # TODO: need to be finished
            self.strategy.current_state = self.strategy.searchForOneState(self.currentAssignment)
            logging.info('Starting at State ' + str(self.strategy.current_state.state_id))
        else:
            logging.error('cannot synthesize a centralized patch')
            pass

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

        # remove any envProps that are sysProps in nextInputs
        nextInputs = ({k:v for k,v in nextInputs.iteritems() if k in currentInputs.keys()})
        # update inputs based on the inputs from the other robot
        currentInputs.update({self.propMappingOldToNew[robot][k]:v for k,v in nextInputs.iteritems()})

        # find if the state is changed
        next_states = self.strategy.findTransitionableStates(currentInputs, from_state= self.strategy.current_state)

        ## This is from executeStrategy.py
        # Make sure we have somewhere to go
        if len(next_states) == 0:
            # Well darn!
            logging.error("Could not find a suitable state to transition to!")
            logging.debug("nextInputs:" + str(nextInputs))
            logging.debug("currentInputs:" + str(currentInputs))
            return

        # See if we're beginning a new transition
        if next_states != self.last_next_states:
            # NOTE: The last_next_states comparison is also to make sure we don't
            # choose a different random next-state each time, in the case of multiple choices
            self.last_next_states = next_states

            # Only allow self-transitions if that is the only option!
            if len(next_states) > 1 and self.strategy.current_state in next_states:
                next_states.remove(self.strategy.current_state)

            next_state = random.choice(next_states)

            if next_state != self.strategy.current_state:
                self.strategy.current_state = next_state
                self.last_next_states = []  # reset
                logging.info('Currently at State ' + str(self.strategy.current_state.state_id))
                logging.debug(str(self.strategy.current_state.getAll(expand_domains=True)))
                logging.debug('---------------------------------------------------------')

        """
        send updated outputs to the robot
        """
        # convert to original prop names and send back to the robot
        nextOutputs = {}
        for prop in self.sysPropList[robot]:
            nextOutputs[prop] = self.strategy.current_state.getAll(expand_domains=True)[self.propMappingOldToNew[robot][prop]]

        return nextOutputs

    def checkIfGoalsAreSatisfied(self):
        """
        Constant check if goal is achieved. If so, terminate cooridation.
        Return true if goals are satisfied and false otherwise.
        """

        return self.sysGoalsCheck.checkViolation(self.strategy.current_state, None)

    def testTriggerSysGoalsSatisfaction(self):
        """
        testing function only: to trigger goal reaching
        """
        # try to trigger goal satisfaction
        self.strategy.current_state = random.choice([x for x in self.strategy.searchForStates({'bob_r3_rc':0, 'bob_r1_rc':0, 'bob_r2_rc':0, 'bob_r5_rc':1, 'bob_r4_rc':0, 'alice_r5_rc':0, 'alice_r1_rc':1, 'alice_r2_rc':0, 'alice_r3_rc':0, 'alice_r4_rc':0, 'bob_r4':0, 'bob_r5':0, 'bob_r1':0, 'bob_r2':0, 'bob_r3':1, 'alice_r5':0, 'alice_r4':0, 'alice_r1':0, 'alice_r3':0, 'alice_r2':1})])

    def updatePatchingRequest(self, functionName, functionPara):
        """
        This function checked if patching request has to be initiated.
        functionName: name of the function (with no parenteses)
        functionPara: parameters of the function

        Update:
        self.patchingRequestReceived object.
        """

        self.patchingRequestReceived = functionName(functionPara)

    def allRobotCentralized(self, para):
        """
        * To be passed to checkChangeinPatchingRequest
        If any robot toggled patching status to true, this function change every robot's patching receival status to true.
        para: parameters. Not used here.

        Output:
        {robot:True} dict if any robot's pathcing status is true and {robot:False} otherwise
        """
        if self.patchingStatus.values().count(True):
            return {robot:True for robot in self.patchingStatus.keys()}
        else:
            return {robot:False for robot in self.patchingStatus.keys()}



if __name__ == "__main__":
    a = CentralExecutor()
    a.run()
    pass
