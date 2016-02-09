#!/usr/bin/env python
"""
This is a helper module for decentralizedPatchingExecutor, handling all the set/get methods.
"""
import socket               # for communication with negotiation monitor
import LTLParser.LTLRegion  # from replace region names in ltl
import LTLParser.LTLcheck   # for retrieve one goal from the group of sysGoals
import ast                  # for parsing spec dict from negtiation monitor
import strategy             # for finding regions from region bits
import numpy                # for generating bit encoding
import parseEnglishToLTL    # for parsing original region name to region bits
import re                   # for parsing regionCompleted_b and region_b to sbit and bit
import copy                 # for making deepcopy of propDict to check envTrans violations for next possible states

# logger for ltlmop
import logging
ltlmop_logger = logging.getLogger('ltlmop_logger')

class MsgHandlerExtensions(object):

    """
    LTLMoP robot object used to communicate with the negotiationMonitor
    """
    def __init__(self, hsub, proj):
        super(MsgHandlerExtensions, self).__init__()

        # check if we are using fastslow:
        self.fastslow = proj.compile_options['fastslow']

        # initialize our variable
        self.robotName = hsub.executing_config.getRobotByName(hsub.executing_config.main_robot).name
        self.loadProjectAndRegions()

    def loadProjectAndRegions(self):
        """
        This function is used to (re)initialize region objects and proj object.
        """
        self.regions = self.proj.rfi.regions # contains decomposed names
        if self.fastslow:
            self.regionCompleted_domain = strategy.Domain("regionCompleted", self.proj.rfi.regions, strategy.Domain.B0_IS_MSB)
        self.region_domain = strategy.Domain("region", self.proj.rfi.regions, strategy.Domain.B0_IS_MSB)

        #find out mapping from new_names to old ones
        for rname, subregs in self.proj.regionMapping.iteritems():
            for newReg in subregs:
                self.newRegionNameToOld[newReg] = rname

        # form regionList with original names
        for region in self.regions:
            self.regionList.append(self.newRegionNameToOld[region.name])

        # for mapping original region name to bit encoding
        self.bitEncode = parseEnglishToLTL.bitEncoding(len(self.regionList), int(numpy.ceil(numpy.log2(len(self.regionList)))))

    def initializeRegionExchange(self, csock, current_region):
        """
        This function obtains the name of the robot and also the list of region and send the info to the negotiation Monitor
        csock: client socket object
        current_region: region object   ---  if current_region = None, then all regions are false
        #!! now sending dict, with init region location included.
        """
        # send region info to the other robot
        if current_region is None:
            self.message_queues[csock].put(self.robotName +'-' + 'regionNames = ' + str({k:False for k in self.regionList}) + '\n')
        else:
            self.message_queues[csock].put(self.robotName +'-' + 'regionNames = ' + str({k:True if self.newRegionNameToOld[current_region.name] == k else False for k in self.regionList}) + '\n')
        ltlmop_logger.info("MSG-Put-region: initialize region info from " + str(self.robotName))

    def updateRobotRegion(self, csock, current_region):
        """
        This function update the region info in the negotiation monitor if the robot is at a next region
        current_region: region object in LTLMoP
        csock: client socket object
        """
        # send current region to the othe robot (csock)
        self.current_region = current_region
        self.message_queues[csock].put(self.robotName + '-' + 'robotLocation = ' + str(self.newRegionNameToOld[current_region.name]) + '\n')
        ltlmop_logger.info("MSG-Put-region: update region info from " + str(self.robotName))

    def updateRobotRegionWithAllClients(self, current_region):
        """
        This function calls updateRobotRegion with all clients in self.clients except self.serv
        """
        for csock in self.clients.values():
            if csock != self.serv:
                self.updateRobotRegion(csock, current_region)

    def initializeActionStatusExchange(self, csock, actionList):
        """
        This function obtains the name of the robot and also the list of region and send the info to the negotiation Monitor
        csock: client socket object
        current_region: region object   ---  if current_region = None, then all regions are false
        #!! now sending dict, with init region location included.
        """
        # send action info to the other robot
        self.message_queues[csock].put(self.robotName +'-' + 'actionNames = ' + str(actionList) + '\n')
        ltlmop_logger.info("MSG-Put-region: initialize action sensor info from " + str(self.robotName))

    def updateRobotActionStatus(self, csock, actionSensorDict):
        """
        This function update the action info in the negotiation monitor if the robot is at a next region
        csock: client socket object
        """
        # send current region to the othe robot (csock)
        self.message_queues[csock].put(self.robotName + '-' + 'updateActionStatus = ' + str(actionSensorDict) + '\n')
        ltlmop_logger.info("MSG-Put-region: update action sensor dict from " + str(self.robotName))

    def updateRobotActionStatusWithAllClients(self, actionSensorDict):
        """
        This function calls updateRobotActionStatus with all clients in self.clients except self.serv
        """
        for csock in self.clients.values():
            if csock != self.serv:
                self.updateRobotActionStatus(csock, actionSensorDict)

    def updateRobotSensors(self, csock, sensorDict):
        """
        This function update the region info in the negotiation monitor if the robot is at a next region
        current_region: region object in LTLMoP
        csock: client socket object
        """
        # send current region to the othe robot (csock)
        self.message_queues[csock].put(self.robotName + '-' + 'robotSensors = ' + str(sensorDict) + '\n')
        ltlmop_logger.info("MSG-Put-region: update sensor dict from " + str(self.robotName))

    def updateRobotSensorsWithAllClients(self, sensorDict):
        """
        This function calls updateRobotRegion with all clients in self.clients except self.serv
        """
        for csock in self.clients.values():
            if csock != self.serv:
                self.updateRobotSensors(csock, sensorDict)

    def initializeCompletedRegionExchange(self, csock, current_region_completed):
        """
        This function sends the list of completed region to the negotiation Monitor
        csock: client socket object
        current_region_completed: region object   ---  if current_region_completed = None, then all regions are false
        #!! now sending dict, with init region location included.
        """
        # send region info to the othe robot (csock)
        if current_region_completed is None:
            self.message_queues[csock].put(self.robotName +'-' + 'regionNames = ' + str({k:False for k in [reg+'_rc' for reg in self.regionList]}) + '\n')
        else:
            self.message_queues[csock].put(self.robotName +'-' + 'regionNames = ' + str({k:True if self.newRegionNameToOld[current_region_completed.name] == k else False for k in [reg+'_rc' for reg in self.regionList]}) + '\n')
        ltlmop_logger.info("MSG-Put-region: initialize regionCompletion info from " + str(self.robotName))

    def updateCompletedRobotRegion(self, csock, current_region):
        """
        This function update the completed region info in the negotiation monitor if the robot is at a next region
        current_region: region object in LTLMoP
        csock: client socket object
        """
        # send current region to the othe robot (csock)
        self.current_region_completed = current_region
        self.message_queues[csock].put(self.robotName + '-' + 'robotLocation = ' + str(self.newRegionNameToOld[current_region.name]+'_rc') + '\n')
        ltlmop_logger.info("MSG-Put-region: update region info from " + str(self.robotName))

    def updateCompletedRobotRegionWithAllClients(self, current_region):
        """
        This function calls updateCompletedRobotRegion with all clients in self.clients except self.serv
        """
        for csock in self.clients.values():
            if csock != self.serv:
                self.updateCompletedRobotRegion(csock, current_region)

    def sendSpec(self, csock, specType, spec, fastslow=False, include_heading=False, current_goal_id=0):
        """
        This function sends the robot's spec to the negotiation monitor according to the specType.
        csock: client socket object
        fastslow: whether we are using completion props
        include_heading: if true then bob_r1 means s.r1, bob_r1 = e.r1 else bob_r1 means e.r1 (fastslow mode)
        current_goal_id: current goal the robot is pursuing (for patching)
        """
        spec = self.sendSpecHelper(specType, spec, fastslow, include_heading, current_goal_id)
        # send sysSafety to negotiation monitor
        self.message_queues[csock].put(self.robotName + "-" + specType + " = '" + spec + "'\n")
        ltlmop_logger.info('MSG-Put-spec: send '+ specType +' from ' + str(self.robotName))

    def sendSpecHelper(self, specType, spec, fastslow=False, include_heading=False, current_goal_id=0):
        """
        This function conducts all the conversion of props and return the spec back to sendSpec for sending.
        (can also just give it to the server directly.)
        """
        # check if the specType is valid
        possibleSpecTypes = ['SysInit', 'SysTrans', 'SysGoals', 'EnvInit', 'EnvTrans', 'EnvGoals', 'SysGoalsOld','WinPos']
        if specType not in possibleSpecTypes:
            raise TypeError('specType must be ' + str(possibleSpecTypes))

        spec = spec.replace(' ', '').replace('\n', '')

        # first replace our region bits to original region name with our robot name
        if fastslow:
            # first replace stay there instances
            spec = LTLParser.LTLRegion.replaceBiimplicationBits(spec, self.regions, self.newRegionNameToOld, self.robotName, fastslow)
            # convert e.sbit to e.robotName_region_rc if include_heading, else to e.robotName_region
            spec = LTLParser.LTLRegion.replaceAllRegionBitsToOriginalName(spec, self.regions, self.regionCompleted_domain, self.newRegionNameToOld, self.robotName, fastslow, include_heading)
            if include_heading:
                # convert s.bit to e.robotName_region or s.bit to s.robotName_region if patching
                spec = LTLParser.LTLRegion.replaceAllRegionBitsToOriginalName(spec, self.regions, self.region_domain, self.newRegionNameToOld, self.robotName, False, False, self.proj.compile_options["multi_robot_mode"] == "d-patching")

            ########################
            ## PATCHING SPECIFICS ##
            ########################
            # only sends the current goal we are pursuing
            if specType == 'SysGoalsOld' or specType == 'SysGoals':
                if not spec.count('[]<>') == 1: # LTLParser doesn't parse single formula with []<> correctly.
                    spec = LTLParser.LTLcheck.ltlStrToList(spec)[current_goal_id]

            # replace e.g. alice_r1 to alice_r1_rc as the meaning changes in the central strategy
            for region in self.regionList:
                for otherRobot in list(set(self.coordinationRequestSent) | set([robot for robot, v in self.coordinationRequest.iteritems() if v])):
                    spec = re.sub('(?<=[! &|(\t\n])e.'+otherRobot+'_'+region+'(?=[ &|)\t\n])', 'e.'+otherRobot+'_'+region+'_rc', spec)

        else:
            # first replace stay there instances
            spec = LTLParser.LTLRegion.replaceBiimplicationBits(spec, self.regions, self.newRegionNameToOld, self.robotName, fastslow, removeSystemProps=True)

            spec = LTLParser.LTLRegion.replaceAllRegionBitsToOriginalName(spec, self.regions, self.region_domain, self.newRegionNameToOld, self.robotName, False)

        return spec

    def parseLocalSpecToGlobalSpec(self, spec):
        """
        This function takes in the local spec and convert all props to the global format.
        """
        # convert e.sbit to e.robotName_region_rc
        spec = LTLParser.LTLRegion.replaceENVsbitToENVRobotNameAndRegionNameRC(spec.replace(" ", ""), self.regions, self.regionCompleted_domain, self.newRegionNameToOld, self.robotName)
        # convert s.bit to e.robotName_region or s.bit to s.robotName_region if patching
        spec = LTLParser.LTLRegion.replaceSYSbitToSYSRobotNameAndRegionName(spec.replace(" ", ""), self.regions, self.region_domain, self.newRegionNameToOld, self.robotName)

        return spec

    def parseLocalSpecListToGlobalSpecList(self, specList):
        """
        This function takes in the local spec list and convert all props to the global format.
        """
        newList = []
        for spec in specList:
            newList.append(self.parseLocalSpecToGlobalSpec(spec))

        return newList

    def sendProp(self, csock, propListType, propDict):
        """
        This function sends environment propsitions with values
        csock: client socket object
        propListType: either 'sys' or 'env'
        propDict    : {propName:propValue}
        """

        propDict = self.sendPropHelper(propListType, propDict)
        #make sure the region names are converted
        if propListType == 'env':
           self.message_queues[csock].put(self.robotName + '-' + 'envPropList = ' + str(propDict) + '\n')
        else:
           self.message_queues[csock].put(self.robotName + '-' + 'sysPropList = ' + str(propDict) + '\n')
        ltlmop_logger.info('MSG-Put-prop: sent '+propListType+'propositions list with value')

    def sendPropHelper(self, propListType, propDict):
        """
        This function conducts all the conversion of props and return the propList back to sendProp for sending.
        (can also just give it to the server directly.)
        """
        propDict = self.convertFromRegionBitsToRegionNameInDict(propListType, propDict)
        return propDict


    def sendNextPossibleEnvStatesToOtherRobot(self, csock, nextStatesArray):
        """
        This function converts possible next states to dict. To be sent to the other robots.
        csock: client socket object

        *Currently we send robotName_region_rc (in the form of robotName_region) to the other robots
        """
        stateArray = []
        #ltlmop_logger.debug('00000000000000')
        for nextState in nextStatesArray:
            #ltlmop_logger.debug("next state includes " + str(nextState.state_id))
            propDict = {}
            propDict.update(self.convertFromRegionBitsToRegionNameInDict('env', nextState.getInputs(expand_domains=True)))
            # we are not sending sysProps anymore
            #propDict.update(self.convertFromRegionBitsToRegionNameInDict('sys', nextState.getOutputs(expand_domains=True)))

            for propKey, propValue in propDict.iteritems():
                # rename region_completed props to without _rc
                if '_rc'in propKey:
                    propDict[propKey.replace('_rc', '')] = propDict.pop(propKey)

            # remove sensor props of other robots
            propDict = {k:v for k, v in propDict.iteritems() for robot in self.robotInRange if robot not in k}

            # add our name back to sensors not having my name
            propDict = {k if self.robotName in k else self.robotName+'_'+k:v for k, v in propDict.iteritems()}

            # append robot name to all props
            if propDict not in stateArray:
                stateArray.append(copy.deepcopy(propDict))

        #ltlmop_logger.debug("stateArray:" + str(stateArray))

        #ltlmop_logger.debug("stateArray:" + str(stateArray))
        self.message_queues[csock].put(self.robotName + '-' + 'nextPossibleStates = ' + str(stateArray) + '\n')
        #ltlmop_logger.info('MSG-Put-nextStates: sent next possible states prop dict')

    def sendNextPossibleEnvStatesToOtherRobotToAllClients(self, nextStatesArray):
        """
        This function takes in the possible next states and send to all clients except myself.
        """
        for csock in self.clients.values():
            if csock != self.serv:
                self.sendNextPossibleEnvStatesToOtherRobot(csock, nextStatesArray)

    def sendNextPossibleEnvStatesPreparedToOtherRobotToAllClients(self, nextStatesArray):
        """
        The array to send to the other robots are preprocessed. Just send pretty much.
        In each dict we only have the env props.

        *Currently we send robotName_region_rc (in the form of robotName_region) to the other robots
        """
        # remove unrelated props. (not including sysProps. only have our robot's _rc props now)
        possible_next_states_dict_array = []
        for state in nextStatesArray:
            # sending only our _rc props
            ourProps = {k.replace('_rc',''):v for k, v in state.getInputs(expand_domains=True).iteritems() if self.robotName in k}

            if ourProps not in possible_next_states_dict_array:
                possible_next_states_dict_array.append(copy.deepcopy(ourProps))

        #ltlmop_logger.debug("nextStatesArray:" + str(nextStatesArray))
        #ltlmop_logger.debug("possible_next_states_dict_array:" + str(possible_next_states_dict_array))

        for csock in self.clients.values():
            if csock != self.serv:
                self.message_queues[csock].put(self.robotName + '-' + 'nextPossibleStates = ' + str(possible_next_states_dict_array) + '\n')

    def getNextPossibleEnvStatesFromOtherRobots(self):
        """
        This function return next possible states from the other robots
        """
        return self.nextPossibleStatesArray

    def convertFromRegionBitsToRegionNameInDict(self, propListType, propDict):
        """
        Convert regionCompleted_b or region_b in prop dict to real region names.
        * make sure the dict only contains sys or env prop or you will need to run each type once.
        propListType: either 'sys' or 'env'
        propDict    : {propName:propValue}
        """
        if propListType not in ['sys', 'env']:
            ltlmop_logger.error('Please specify your propListType correctly!')
            return

        if propListType == 'sys':
            strategyBitStr = 'region_b'
            suffix = ''
            domain = self.region_domain
        else:
            strategyBitStr = 'regionCompleted_b'
            suffix = '_rc'
            domain = self.regionCompleted_domain

        # send prop
        def decorate_prop(prop):
            """
            replace regionCompleted_b and region_b to sbit and bit
            """
            prop = re.sub(r'region_b(\d+)$', r'bit\1', prop)
            prop = re.sub(r'regionCompleted_b(\d+)$', r'sbit\1', prop)
            return prop

        #make sure the region names are converted
        propDictOnlyReg = {prop:value for prop, value in propDict.iteritems() if strategyBitStr in prop}
        propDict = {prop:value for prop, value in propDict.iteritems() if not strategyBitStr in prop}

        # find region in new name
        targetRegionNew = self.regions[domain.propAssignmentsToNumericValue(propDictOnlyReg)]

        # find reigon in old name
        targetRegionOrig = self.newRegionNameToOld[targetRegionNew.name]

        # append regions to dictionary
        for origReg in self.regionList:
            if origReg == targetRegionOrig:
                propDict.update({self.robotName+'_'+origReg+suffix:True})
            else:
                propDict.update({self.robotName+'_'+origReg+suffix:False})

        return propDict

    def getOutputs(self, envPropDict):
        """
        Communicate with the central executor. Send in the next inputs and ask for the next output.
        envPropDict: dict of env props with values
        """
        envPropDict = self.convertFromRegionBitsToRegionNameInDict('env', envPropDict)

        # get outputs
        outputs = self.findRobotOutputs(self.robotName, envPropDict)
        if outputs is None:
            # cannot find a suitable state to transition to!
            return []
        else:
            # combine all boolean region props into one
            sys_region = [k.replace(self.robotName+'_', '') for k, v in outputs.iteritems() if k.replace(self.robotName+'_', '') in self.regionList and v]

            # find all other sysProps and exclude the region ones
            outputs = {k:v for k, v in outputs.iteritems() if not k.replace(self.robotName+'_', '') in self.regionList}

            # append the region object into the outputs dict
            if not sys_region:
                #temporarily use the old one
                ltlmop_logger.warning('sys_region outputs are not correct.Using old one:' + str(sys_region))
                outputs['region'] = self.prev_outputs['region']
            elif len(self.proj.regionMapping[sys_region[0]]) == 1:
                outputs['region'] = self.regions[self.proj.rfi.indexOfRegionWithName(self.proj.regionMapping[sys_region[0]][0])]
            else:
                ltlmop_logger.warning('The regions are decomposed. We might want to do this differently')

            self.prev_outputs = outputs

            return outputs

    def sendCoordinationRequest(self, csock, coordinationRequest):
        """
        This function sends coorindation request to csock.
        csock: client socket object
        coordinationRequest: True if we want to initiate coorination with the other robot.
        """
        self.message_queues[csock].put(self.robotName + '-' + 'coordinationRequest = ' + str(coordinationRequest) +  '\n')

        #track who we have sent reqeust to
        #if self.robotAddresses.keys()[self.robotAddresses.values().index(csock.getpeername())] not in self.coordinationRequestSent:
        #    self.coordinationRequestSent.append(self.robotAddresses.keys()[self.robotAddresses.values().index(csock.getpeername())])

        ltlmop_logger.info('MSG-Put-Status: set coorindation status to ' + str(coordinationRequest) + ' to ' + str(csock))

    # def getCoordinationRequest(self):
    #     """
    #     This function returns coordinationRequest dict.
    #     """
    #     return self.coordinationRequest

    def checkIfCoordinationRequestIsRecevied(self):
        """
        This function returns true if some robots request for coorindation and false otherwise
        """
        return True in self.coordinationRequest.values()

    def sendRestartStatusToAllCoordinatingRobots(self):
        """
        This function sends restart status to all robots coordinating.
        """
        for robot in self.readyToRestart.keys(): #keys of readyToRestart are robots in the cooridination
            ltlmop_logger.debug("self.clients[robot]:" + " robot:" + str(robot) + ' csock:' + str(self.clients[robot]))
            self.setRestartStatusToTrue(self.clients[robot])

    def setRestartStatusToTrue(self, csock):
        """
        This function sets the restart status to true.
        csock: client socket object
        """
        self.message_queues[csock].put(self.robotName + '-' + 'restartStatus = ' + str(True) +  '\n')
        ltlmop_logger.info('MSG-Put-Status: set restart status to ' + str(True))

    def checkRestartStatus(self):
        """
        This function checks if all robots are ready to restart.
        csock: client socket object
        """
        return not (False in self.readyToRestart.values())

    def sendCentralizedExecutionStatus(self, csock):
        """
        This function checks if centralized execution has started.
        True if ready. False is not and None has not started/ended.
        csock: client socket object
        """
        self.message_queues[csock].put(self.robotName + '-' + 'centralizedExecutionStatus = ' + str(self.centralizedExecutionStatus) +  '\n')
        ltlmop_logger.info('MSG-Put-Status: set coodination status to ' + str(self.centralizedExecutionStatus))

    def checkIfOtherRobotsAreReadyToExecuteCentralizedStrategy(self):
        """
        This function checks if all robots coorindating are ready to work together.
        """
        if self.robotStatusOnCentralizedStrategyExecution:
            #first update latest info from temp
            for robot, v in self.tempRobotStatusOnCentralizedStrategyExecution.iteritems():
                self.robotStatusOnCentralizedStrategyExecution[robot] = v

            # reset self.tempRobotStatusOnCentralizedStrategyExecution
            self.tempRobotStatusOnCentralizedStrategyExecution = {}

            return not (False in self.robotStatusOnCentralizedStrategyExecution.values())
        else: # dict is empty. no centralized execution going on
            return False

    def getCentralizedExecutionStatus(self):
        """
        Get the current centralized strategy execution status
        True: centrlized strategy is going on
        False: we are preparing for centralized execution
        None: Centralized execution has ended/ not started
        """
        return self.centralizedExecutionStatus

    def setPauseForControllerSynthesis(self, csock, status):
        """
        This function notifiies the other robots to stay in place and wait for synthesis together.
        """
        self.message_queues[csock].put(self.robotName + '-' + 'pauseForSynthesis = ' + str(status) +  '\n')

    def getPauseForControllerSynthesis(self):
        """
        This function gets the status to whether stay in place and wait for synthesis.
        """
        return True in self.pauseForSynthesis.values()
