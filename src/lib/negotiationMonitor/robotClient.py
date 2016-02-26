#robotClient.py
import socket               # for communication with negotiation monitor
import logging
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
#logging.basicConfig(level=logging.DEBUG)
#logger = logging.getLogger(__name__)

class RobotClient:

    """
    LTLMoP robot object used to communicate with the negotiationMonitor
    """
    def __init__(self, hsub, proj):
        ADDR = ("localhost",6501)
        self.BUFSIZE = 2000000

        # check if we are using fastslow:
        self.fastslow = proj.compile_options['fastslow']
        self.proj = proj

        # initialize our variable
        self.robotName = ''
        self.loadProjectAndRegions(proj)

        #connect to the server
        self.clientObject = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.clientObject.connect((ADDR))

        #send out initial info
        self.initializeRegionExchange(hsub)
        if proj.compile_options['include_heading']:
            self.initializeCompletedRegionExchange()

        # track if spec is requested
        self.specRequestFromOther = [] # list of spec requested

        # storing aut execution
        self.prev_outputs = None

    def loadProjectAndRegions(self,proj):
        """
        This function is used to (re)initialize region objects and proj object.
        """
        self.proj = proj
        self.regions    = self.proj.rfi.regions # contains decomposed names
        if self.fastslow:
            self.regionCompleted_domain = strategy.Domain("regionCompleted",  proj.rfi.regions, strategy.Domain.B0_IS_MSB)
        self.region_domain = strategy.Domain("region",  proj.rfi.regions, strategy.Domain.B0_IS_MSB)

        #find out mapping from new_names to old ones
        self.newRegionNameToOld = {}
        for rname, subregs in proj.regionMapping.iteritems():
            for newReg in subregs:
                self.newRegionNameToOld[newReg] = rname

        # form regionList with original names
        self.regionList = []
        for region in self.regions:
            self.regionList.append(self.newRegionNameToOld[region.name])

        # for mapping original region name to bit encoding
        self.bitEncode = parseEnglishToLTL.bitEncoding(len(self.regionList), int(numpy.ceil(numpy.log2(len(self.regionList)))))

    def initializeRegionExchange(self, hsub):
        """
        This function obtains the name of the robot and also the list of region and send the info to the negotiation Monitor
        hsub:         self.hsub from LTLMoP
        """
        self.robotName  = hsub.executing_config.getRobotByName(hsub.executing_config.main_robot).name
        
        # send region info to negotiation monitor
        self.clientObject.send(self.robotName +'-' + 'regionList = ' + str(self.regionList) + '\n')
        ltlmop_logger.info("ROBOTCLIENT: initialize region info from " + str(self.robotName))
        
    def updateRobotRegion(self, current_region):
        """
        This function update the region info in the negotiation monitor if the robot is at a next region
        current_region: region object in LTLMoP
        """
        # send current region to negotiation monitor       
        self.clientObject.send(self.robotName + '-' + 'regionName = ' + str(self.newRegionNameToOld[current_region.name]) + '\n')
        ltlmop_logger.info("ROBOTCLIENT: update region info from " + str(self.robotName))

    def initializeCompletedRegionExchange(self):
        """
        This function sends the list of completed region to the negotiation Monitor
        hsub:         self.hsub from LTLMoP
        """
        # send region info to negotiation monitor
        self.clientObject.send(self.robotName +'-' + 'regionList = ' + str([reg+'_rc' for reg in self.regionList]) + '\n')
        ltlmop_logger.info("ROBOTCLIENT: initialize region info from " + str(self.robotName))

    def updateCompletedRobotRegion(self, current_region):
        """
        This function update the completed region info in the negotiation monitor if the robot is at a next region
        current_region: region object in LTLMoP
        """
        # send current region to negotiation monitor
        self.clientObject.send(self.robotName + '-' + 'regionName = ' + str(self.newRegionNameToOld[current_region.name]+'_rc') + '\n')
        ltlmop_logger.info("ROBOTCLIENT: update region info from " + str(self.robotName))

    def closeConnection(self):
        """
        This function closes the connection with the negotiation monitor.
        """    
        self.clientObject.close()
        ltlmop_logger.info('ROBOTCLIENT: connection to the negotiation monitor is now closed')
   
    def sendSpec(self, specType, spec, fastslow=False, include_heading=False, current_goal_id=0):
        """
        This function sends the robot's spec to the negotiation monitor according to the specType.
        fastslow: whether we are using completion props
        include_heading: if true then bob_r1 means s.r1, bob_r1 = e.r1 else bob_r1 means e.r1 (fastslow mode)
        current_goal_id: current goal the robot is pursuing (for patching)
        """
        # check if the specType is valid
        possibleSpecTypes = ['SysInit','SysTrans','SysGoals','EnvInit','EnvTrans','EnvGoals','SysGoalsOld']
        if specType not in possibleSpecTypes:
            raise TypeError('specType must be ' + str(possibleSpecTypes))
        
        #spec = spec.replace('\t',"").replace(' ','').replace('\n','')
        spec = spec.replace(' ','').replace('\n','')
        ltlmop_logger.log(1,'spec:' + str(spec))

        # first replace our region bits to original region name with our robot name
        if fastslow:
            # first replace stay there instances
            spec = LTLParser.LTLRegion.replaceBiimplicationBits(spec, self.regions, self.newRegionNameToOld, self.robotName, fastslow)

            if specType == 'SysGoals':
                spec = LTLParser.LTLRegion.replaceSYSbitToENVRobotNameAndRegionName(spec, self.regions, self.region_domain, self.newRegionNameToOld, self.robotName)
                if include_heading:
                    spec = LTLParser.LTLRegion.replaceENVsbitToENVRobotNameAndRegionNameRC(spec, self.regions, self.regionCompleted_domain, self.newRegionNameToOld, self.robotName)
                else:
                    spec = LTLParser.LTLRegion.replaceENVsbitToENVRobotNameAndRegionName(spec, self.regions, self.regionCompleted_domain, self.newRegionNameToOld, self.robotName)
            else:
                # convert e.sbit to e.robotName_region_rc if include_heading, else to e.robotName_region
                ltlmop_logger.log(4,'BEFORE bit Change:' + spec)
                spec =  LTLParser.LTLRegion.replaceAllRegionBitsToOriginalName(spec, self.regions, self.regionCompleted_domain, self.newRegionNameToOld, self.robotName, fastslow, include_heading)
                ltlmop_logger.log(4,'AFTER bit Change:' + spec)

                if include_heading:
                    # convert s.bit to e.robotName_region or s.bit to s.robotName_region if patching
                    spec =  LTLParser.LTLRegion.replaceAllRegionBitsToOriginalName(spec, self.regions, self.region_domain, self.newRegionNameToOld, self.robotName, False, False, self.proj.compile_options["multi_robot_mode"] == "patching")

            ########################
            ## PATCHING SPECIFICS ##
            ########################
            if self.proj.compile_options['neighbour_robot'] and self.proj.compile_options["multi_robot_mode"] == "patching":
                # only sends the current goal we are pursuing
                if specType == 'SysGoalsOld':
                    if not spec.count('[]<>') == 1: # LTLParser doesn't parse single formula with []<> correctly.
                        spec = LTLParser.LTLcheck.ltlStrToList(spec)[current_goal_id]

                # replace e.g. alice_r1 to alice_r1_rc as the meaning changes in the central strategy
                for region in self.regionList:
                    for otherRobot in self.proj.otherRobot:
                        spec = re.sub('(?<=[! &|(\t\n])e.'+otherRobot+'_'+region+'(?=[ &|)\t\n])','e.'+otherRobot+'_'+region+'_rc',spec)

            ###########################
            ## NEGOTIATION SPECIFICS ##
            ###########################
            # Before sending EnvTrans to nego Monitor, we need to remove
            # spec relating only the system robot.
            elif self.proj.compile_options['neighbour_robot'] and self.proj.compile_options["multi_robot_mode"] == "negotiation":
                if fastslow and specType == 'EnvTrans':
                    # for more than two robots. all spec gone because it's doing one at a time
                    spec = '&'.join(LTLParser.LTLcheck.excludeSysRobotOnlySpecList(LTLParser.LTLcheck.ltlStrToList(spec), self.robotName, [self.robotName]+ self.proj.otherRobot))
                    ltlmop_logger.log(4, 'Removing spec about itself:' + spec)

                # convert customProp from prop to robot_prop
                for prop in self.proj.all_customs:
                    spec = re.sub('(?<=[! &|(\t\n])s.'+prop+'(?=[ &|)\t\n])','e.'+self.robotName+'_'+prop,spec)
                ltlmop_logger.log(6, 'Custom Prop modified:' + spec)

                # convert actProp from prop to robot_Actprop
                # convert actProp_ac from prop_ac to robot_prop
                for prop in self.proj.enabled_actuators:
                    spec = re.sub('(?<=[! &|(\t\n])s.'+prop+'(?=[ &|)\t\n])','e.'+self.robotName+'_Act'+prop,spec)
                    spec = re.sub('(?<=[! &|(\t\n])e.'+prop+'_ac(?=[ &|)\t\n])','e.'+self.robotName+'_'+prop,spec)

                ltlmop_logger.log(6, 'Actuator Prop modified:' + spec)

        else:
            spec =  LTLParser.LTLRegion.replaceAllRegionBitsToOriginalName(spec, self.regions, self.region_domain, self.newRegionNameToOld, self.robotName, False)

        # send sysSafety to negotiation monitor
        self.clientObject.send(self.robotName + "-" + specType + " = '" + spec + "'\n")
        ltlmop_logger.info('ROBOTCLIENT: send '+ specType +' from ' + str(self.robotName))
        
    
    def requestSpec(self, specType):
        """
        This function requests the system guarantees of the other robots from the negotiation monitor.
        OUTPUT:
        specToAppend: ltl formula ready to append to envTrans
        """
        # check if the specType is valid
        possibleSpecTypes = ['SysTrans','SysGoals','EnvTrans','EnvGoals']
        if specType not in possibleSpecTypes:
            raise TypeError('specType must be ' + str(possibleSpecTypes))
            
        specToAppend = ""
        
        #TODO: need to get full picture of all robots. The current solution only deals with two robots
        ltlmop_logger.info('ROBOTCLIENT: request '+ specType + ' of other robots')
        while not len(specToAppend):
            self.clientObject.send(self.robotName + '-' + specType +' = ' + "''" '\n')

            #receive info (also check if msg is complete)
            bufferData = self.clientObject.recv(self.BUFSIZE)
            fullMsg = bufferData
            obtainFullMsg = False
            while not obtainFullMsg:
                try:
                    ast.literal_eval(fullMsg)
                    obtainFullMsg = True
                except:
                    bufferData = self.clientObject.recv(self.BUFSIZE)
                    fullMsg += bufferData
                    ltlmop_logger.log(2, "MSG is not completed. Getting more parts.")

            ltlmop_logger.log(2, specType + ":" + fullMsg)
            SpecDict = ast.literal_eval(fullMsg)

            requestingRobot = self.getNegotiationInitiatingRobot()
            joiningRobot    = self.checkNegotiationStatus()
            for robot, spec in SpecDict.iteritems():
                #self.robotName = 'alice' #TODO: remove this later
                if self.robotName != robot and (robot == requestingRobot or robot == joiningRobot):

                    # change region props with our name to region bits (parseEnglishToLTL?)              
                    specToAppend += LTLParser.LTLRegion.replaceRobotNameWithRegionToBits(spec, self.bitEncode, self.robotName, self.regionList, self.fastslow, self.proj.compile_options['include_heading'])

        # replace actuator names
        # e.robotName_Act(actuatorName) = sys prop
        # e.robotName_(actuatorName_ac) = _ac prop
        # specToAppend = specToAppend.replace('e.'+self.robotName+'_Act','s.').replace('e.'+self.robotName+'_','e.')
        for prop in self.proj.enabled_actuators:
            specToAppend = re.sub('(?<=[! &|(\t\n])e.'+self.robotName+'_Act'+prop+'(?=[ &|)\t\n])','s.'+prop,specToAppend)
            specToAppend = re.sub('(?<=[! &|(\t\n])e.'+self.robotName+'_'+prop+'(?=[ &|)\t\n])','e.'+prop+'_ac',specToAppend)

        # replace custom prop
        for prop in self.proj.all_customs:
            specToAppend = re.sub('(?<=[! &|(\t\n])e.'+self.robotName+'_'+prop+'(?=[ &|)\t\n])','s.'+prop,specToAppend)

        return specToAppend 
        
    def requestRegionInfo(self):
        """
        This function request the region dict of all robots.
        OUTPUT: 
        robotRegionStatus: dict containing all region info
        """
        self.clientObject.send(self.robotName +'-' + 'sensorUpdate = ' + "''" + '\n')
        robotRegionStatus = ast.literal_eval(self.clientObject.recv(self.BUFSIZE))
        
        return robotRegionStatus
        
    def updateStrategyStatus(self,realizable):
        """
        This function update the current controller status to the negotiation monitor.
        INPUT: 
        realizable: True or False
        """
        self.clientObject.send(self.robotName +'-' + 'updateStrategyStatus = ' + str(realizable) + '\n')
        
    def requestStrategyStatus(self):
        """
        This function requests the controller status of the other robots from the negotiation monitor.
        OUTPUT:
        realizable: dict of boolean. realizable['rob1'] = True
        """
        self.clientObject.send(self.robotName + '-' + 'requestStrategyStatus = ' + "''" + '\n')
        ltlmop_logger.info('ROBOTCLIENT: request strategy status of other robots')

        #receive info
        realizable = ast.literal_eval(self.clientObject.recv(self.BUFSIZE))
        ltlmop_logger.debug(realizable)
        
        return realizable
    
    def checkRequestSpec(self):
        """
        This function check if our specification is currently requested by the other robot
        """
        self.clientObject.send(self.robotName + '-' + 'requestSpecStatus = ' + "''" +  '\n')
        #ltlmop_logger.info('ROBOTCLIENT: check request spec status of other robots')
        
        #receive info
        self.specRequestFromOther = ast.literal_eval(self.clientObject.recv(self.BUFSIZE))
        #ltlmop_logger.debug(self.specRequestFromOther)

    def getNegotiationInitiatingRobot(self):
        """
        This function gets the robot that initiates negotiation.
        """
        self.clientObject.send(self.robotName + '-' + 'negotiationStatus = ' + "''" + '\n')

        #receive info
        originalStr = self.clientObject.recv(self.BUFSIZE)
        bufferData = originalStr.split(';').pop()
        ltlmop_logger.log(2,"negotiationStatus:" + str(bufferData.split('-')[0]))
        bufferData = bufferData.split('-')[1]
        ltlmop_logger.log(2,"negotiationInitiator:" + str(bufferData))
        negotiationInitiator = ast.literal_eval(bufferData)
        return negotiationInitiator
   
    def checkNegotiationStatus(self):
        """
        This function check the current negotiation status.
        None     : no negotiation has started.
        robotName: This robot should resynthesize the specification with the spec snippets from the other robot.
        True     : negotiation is completed.
        False    : negotiation failed. Alternatives seeked.
        """
        self.clientObject.send(self.robotName + '-' + 'negotiationStatus = ' + "''" + '\n')
        
        #receive info
        originalStr = self.clientObject.recv(self.BUFSIZE)
        bufferData = originalStr.split(';').pop()
        bufferData = bufferData.split('-')[0]
        negotiationStatus = ast.literal_eval(bufferData)
        return negotiationStatus
        
    def setNegotiationStatus(self, status):
        """
        This function set the current negotiation status.
        None     : no negotiation has started.
        robotName: This robot should resynthesize the specification with the spec snippets from the other robot.
        True     : negotiation is completed.
        False    : negotiation failed. Alternatives seeked.
        """

        self.clientObject.send(self.robotName + '-' + 'negotiationStatus = ' + str(status) + '\n')
        ltlmop_logger.info('ROBOTCLIENT: negotiation status set')
        
    def getViolationTimeStamp(self, otherRobotName):
        """
        This function gets the violation time stamp.
        """
        self.clientObject.send(self.robotName + '-' + 'violationTimeStamp = ' + "''" + '\n')
        timeStamp = ast.literal_eval(self.clientObject.recv(self.BUFSIZE))

        return timeStamp[otherRobotName]
        
    def setViolationTimeStamp(self, timeStamp):
        """
        This function sets the violation time stamp.
        """
        self.clientObject.send(self.robotName + '-' + 'violationTimeStamp = ' + str(timeStamp) + '\n')

    def sendProp(self, propListType, propDict):
        """
        This function sends environment propsitions with values
        propListType: either 'sys' or 'env'
        propDict    : {propName:propValue}
        """
        propDict = self.convertFromRegionBitsToRegionNameInDict(propListType, propDict)
        #make sure the region names are converted
        if propListType == 'env':
            self.clientObject.send(self.robotName + '-' + 'envPropList = ' + str(propDict) + '\n')
        else:
            self.clientObject.send(self.robotName + '-' + 'sysPropList = ' + str(propDict) + '\n')
        ltlmop_logger.info('ROBOTCLIENT: sent '+propListType+'propositions list with value')

    def sendNextPossibleEnvStatesToOtherRobot(self, nextStatesArray):
        """
        This function converts possible next states to dict. To be sent to the other robots.
        """
        stateArray = []
        #ltlmop_logger.debug('00000000000000')
        for nextState in nextStatesArray:
            #ltlmop_logger.debug("next state includes " + str(nextState.state_id))
            propDict = {}
            propDict.update(self.convertFromRegionBitsToRegionNameInDict('env', nextState.getInputs(expand_domains=True)))
            # we are not sending sysProps anymore
            #propDict.update(self.convertFromRegionBitsToRegionNameInDict('sys', nextState.getOutputs(expand_domains=True)))

            # rename region_completed props to without _rc
            for propKey, propValue in propDict.iteritems():
                if '_rc'in propKey:
                    propDict[propKey.replace('_rc','')] = propDict.pop(propKey)

            # append robot name to all props
            stateArray.append(copy.deepcopy(propDict))

        #ltlmop_logger.debug("stateArray:" + str(stateArray))
        self.clientObject.send(self.robotName + '-' + 'nextPossibleStates = ' + str(stateArray) + '\n')
        #ltlmop_logger.info('ROBOTCLIENT: sent next possible states prop dict')

    def requestNextPossibleEnvStatesFromOtherRobot(self):
        """
        This function requests next possible states from the other robots
        """
        self.clientObject.send(self.robotName + '-' + 'nextPossibleStates = ' + "''" + '\n')
        #ltlmop_logger.info('ROBOTCLIENT: requested next possible states prop dict')

        #receive info
        msg = ""
        nextPossibleStates = None

        while nextPossibleStates is None:
            try:
                msg += self.clientObject.recv(self.BUFSIZE)
                nextPossibleStates = ast.literal_eval(msg)
            except:
                pass
        #ltlmop_logger.debug("nextPossibleStates:" + str(nextPossibleStates))
        return nextPossibleStates


    def convertFromRegionBitsToRegionNameInDict(self, propListType, propDict):
        """
        Convert regionCompleted_b or region_b in prop dict to real region names.
        * make sure the dict only contains sys or env prop or you will need to run each type once.
        propListType: either 'sys' or 'env'
        propDict    : {propName:propValue}
        """
        if propListType not in ['sys','env']:
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
        self.clientObject.send(self.robotName + '-' + 'automatonExecution = ' + str(envPropDict) + '\n')

        # receive outputs
        outputs = ast.literal_eval(self.clientObject.recv(self.BUFSIZE))
        if outputs is None:
            # cannot find a suitable state to transition to!
            return []
        else:
            # combine all boolean region props into one
            sys_region = [k.replace(self.robotName+'_','') for k, v in outputs.iteritems() if k.replace(self.robotName+'_','') in self.regionList and v]


            # find all other sysProps and exclude the region ones
            outputs = {k:v for k, v in outputs.iteritems() if not k.replace(self.robotName+'_','') in self.regionList}

            # append the region object into the outputs dict
            if not sys_region:
                #temporarily use the old one
                ltlmop_logger.warning('sys_region outputs are not correct.Using old one:' + str(sys_region))
                outputs['region'] = self.prev_outputs['region']
            elif len(self.proj.regionMapping[sys_region[0]]) == 1:
                outputs['region'] = self.regions[self.proj.rfi.indexOfRegionWithName(self.proj.regionMapping[sys_region[0]][0])]
            else:
                ltlmop_logger.warning('The regions are decomposed. We might want to do this differently')

            self.prev_outputs = copy.deepcopy(outputs)
            return outputs

    def setCoordinationStatus(self, patchingStatus):
        """
        This function sets the patching status.
        """
        self.clientObject.send(self.robotName + '-' + 'patchingStatus = ' + str(patchingStatus) +  '\n')
        ltlmop_logger.info('ROBOTCLIENT: set coorindation status to ' + str(patchingStatus))

    def checkCoordinationRequest(self):
        """
        This function checks if patching is initiated.
        """
        self.clientObject.send(self.robotName + '-' + 'patchingRequest = ' + "''" +  '\n')
        status = ast.literal_eval(self.clientObject.recv(self.BUFSIZE))

        return status

    def setRestartStatus(self):
        """
        This function sets the restart status.
        """
        self.clientObject.send(self.robotName + '-' + 'restartStatus = ' + str(True) +  '\n')
        ltlmop_logger.info('ROBOTCLIENT: set restart status to ' + str(True))

    def checkRestartStatus(self):
        """
        This function checks if all robots are ready to restart.
        """
        self.clientObject.send(self.robotName + '-' + 'restartStatus = ' + "''" +  '\n')
        status = ast.literal_eval(self.clientObject.recv(self.BUFSIZE))

        return status

    def getCentralizedExecutionStatus(self):
        """
        This function checks if centralized execution has started.
        True if started. False is waiting and None has not started/ended.
        """
        self.clientObject.send(self.robotName + '-' + 'centralizedExecutionStatus = ' + "''" +  '\n')
        status = ast.literal_eval(self.clientObject.recv(self.BUFSIZE))

        return status

    def updateRobotSensors(self, sensorDict):
        """
        This function update the sensor dict in the negotiation monitor if the robot's sensors changes
        sensorDict: dictionary of sensors.
        """
        # send current region to the othe robot (csock)
        self.clientObject.send(self.robotName + '-' + 'robotSensors = ' + str(sensorDict) + '\n')
        #ltlmop_logger.info("MSG-Put-region: update sensor dict from " + str(self.robotName))

    def getRobotSensorsStatus(self):
        # send current region to the othe robot (csock)
        self.clientObject.send(self.robotName + '-' + 'robotSensors = ' + "''" + '\n')
        #ltlmop_logger.info("MSG-Put-region: get sensor dict for " + str(self.robotName))
        sensorDict = ast.literal_eval(self.clientObject.recv(self.BUFSIZE))
        return sensorDict


