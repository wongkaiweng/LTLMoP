import re

import LTLParser.LTLcheck
import LTLParser.LTLFormula
import time
import copy

# logger for ltlmop
import logging
ltlmop_logger = logging.getLogger('ltlmop_logger')

class ExecutorDecentralizedPatchingExtensions(object):
    """ Extensions to Executor to allow for glocal patching. """

    def __init__(self):
        super(ExecutorDecentralizedPatchingExtensions, self).__init__()
        ltlmop_logger.info("Initializing decentralized patching extensions...")

    # %%%%%%%%%% d-patching %%%%%%%%%%%%%%%%% #
    def prepareForCentralizedStrategySnippetToOtherRobots(self, csockList):
        """
        This function prepares for the synthesis of centralized strategy.
        (modified from as prepareForCentralizedStrategy function)
        csockList: list of sockets to send snippets to
        """

        # send cooridination status
        for csock in csockList:
            self.dPatchingExecutor.sendCoordinationRequest(csock, True)

        # load spec
        mySpec = self.loadSpecObjectFromFile()

        # send winning positions
        winPosStr = self.extractWinningPositions()
        for csock in csockList:
            self.dPatchingExecutor.sendSpec(csock, 'WinPos', winPosStr, fastslow=True, include_heading=True)

        # send spec
        for specType, specStr in mySpec.iteritems():
            if specType not in ['SysInit', 'SysTrans', 'SysGoals', 'EnvInit', 'EnvTrans', 'EnvGoals']:
                continue

            # tell robClient the current goal we are pursuing
            if specType == 'SysGoals':
                #specNewStr = self.extractCurrentLivenessWithWinningPositions(self.strategy.current_state.goal_id)
                for csock in csockList:
                    #self.dPatchingExecutor.sendSpec(csock, specType, specNewStr, fastslow=True, include_heading=True)
                    self.dPatchingExecutor.sendSpec(csock, specType, specStr, fastslow=True, include_heading=True, current_goal_id=int(self.prev_z))

                # print current goal pursuing
                #currentGoalLTL = (str(LTLParser.LTLcheck.ltlStrToList(specStr)[int(self.strategy.current_state.goal_id)]) if not specStr.count('[]<>') == 1 else specStr)
                #self.postEvent("D-PATCH", "Current goal is " + self.proj.specText.split('\n')[self.tracebackTree['SysGoals'][int(self.strategy.current_state.goal_id)]-1])

            elif specType == 'EnvTrans':
                # remove violated lines. Can be changed to doing sth else
                #HACK: converting to global names here. should be done in sendSpecHelper but parsing is not quite right for regions
                specStr = self.dPatchingExecutor.parseLocalSpecToGlobalSpec(specStr)

                # first make sure all bits can be parsed in violated list
                violatedList = LTLParser.LTLRegion.addParenthesisToBitsGroupInLTLList(list(set(self.violated_spec_list + self.possible_states_violated_spec_list)),\
                                self.proj.rfi.regions)
                violatedList = self.dPatchingExecutor.parseLocalSpecListToGlobalSpecList(violatedList)
                # remove violated spec
                #specNewStr = self.filterAndExcludeSpec(violatedList, specStr)
                # OR
                # remove spec relating the coordinating robots
                #specNewStr = self.filterAndExcludeSpecOfCoordinatingRobots(violatedList, specStr)
                #ltlmop_logger.debug("specNewStr:" + str(specNewStr))
                specNewStr = specStr
                for csock in csockList:
                    self.dPatchingExecutor.sendSpec(csock, specType, specNewStr, fastslow=True, include_heading=True)

            else:
                for csock in csockList:
                    self.dPatchingExecutor.sendSpec(csock, specType, specStr, fastslow=True, include_heading=True)

        # send also the old sysGoals
        for csock in csockList:
            self.dPatchingExecutor.sendSpec(csock, 'SysGoalsOld', mySpec['SysGoals'], fastslow=True, include_heading=True, current_goal_id=int(self.prev_z))

            # send prop
            self.dPatchingExecutor.sendProp(csock, 'env', self.sensor_strategy.getInputs(expand_domains=True))

        #self.robClient.sendProp('env', self.strategy.current_state.getInputs(expand_domains = True))
        ltlmop_logger.debug("Using sensor state. Should be the violation one" + str(self.sensor_strategy.getInputs()))
        # TODO: Here we assume we are using bits
        # replace sys init with env init for regions
        sysProps = self.strategy.current_state.getOutputs(expand_domains=True)
        sysProps = {sProp:False if 'region_b' in sProp else sValue for sProp, sValue in sysProps.iteritems()}

        for eProp, eValue in self.sensor_strategy.getInputs(expand_domains=True).iteritems():
            if 'regionCompleted_b' in eProp and eValue:
                sysProps[eProp.replace('regionCompleted_b', 'region_b')] = eValue

        for csock in csockList:
            self.dPatchingExecutor.sendProp(csock, 'sys', sysProps)

    def prepareForCentralizedStrategySnippetToSelf(self):
        """
        This function prepares for the synthesis of centralized strategy.
        (modified from as prepareForCentralizedStrategy function)
        """

        # load spec
        mySpec = self.loadSpecObjectFromFile()

        # send winning positions
        winPosStr = self.extractWinningPositions()
        self.dPatchingExecutor.winPos[self.dPatchingExecutor.robotName] = \
            self.dPatchingExecutor.sendSpecHelper('WinPos', winPosStr, fastslow=True, include_heading=True)

        # send spec
        for specType, specStr in mySpec.iteritems():
            if specType not in ['SysInit', 'SysTrans', 'SysGoals', 'EnvInit', 'EnvTrans', 'EnvGoals']:
                continue

            # tell robClient the current goal we are pursuing
            if specType == 'SysGoals':
                #specNewStr = self.extractCurrentLivenessWithWinningPositions(self.strategy.current_state.goal_id)
                #self.dPatchingExecutor.spec[specType][self.dPatchingExecutor.robotName] = \
                #    self.dPatchingExecutor.sendSpecHelper(specType, specNewStr, fastslow=True, include_heading=True)
                self.dPatchingExecutor.spec[specType][self.dPatchingExecutor.robotName] = \
                    self.dPatchingExecutor.sendSpecHelper(specType, specStr, fastslow=True, include_heading=True, current_goal_id=int(self.prev_z))

                # print current goal pursuing
                self.postEvent("D-PATCH", "Current goal is " + self.proj.specText.split('\n')[self.tracebackTree['SysGoals'][int(self.prev_z)]-1])

            elif specType == 'EnvTrans':
                # remove violated lines. Can be changed to doing sth else
                #HACK: converting to global names here. should be done in sendSpecHelper but parsing is not quite right for regions
                specStr = self.dPatchingExecutor.parseLocalSpecToGlobalSpec(specStr)
                violatedList = LTLParser.LTLRegion.addParenthesisToBitsGroupInLTLList(list(set(self.violated_spec_list + self.possible_states_violated_spec_list)),\
                                self.proj.rfi.regions)
                violatedList = self.dPatchingExecutor.parseLocalSpecListToGlobalSpecList(violatedList)
                # remove violated spec
                #specNewStr = self.filterAndExcludeSpec(violatedList, specStr)
                # OR
                # remove spec relating the coordinating robots
                #specNewStr = self.filterAndExcludeSpecOfCoordinatingRobots(violatedList, specStr)
                specNewStr = specStr
                self.dPatchingExecutor.spec[specType][self.dPatchingExecutor.robotName] = \
                    self.dPatchingExecutor.sendSpecHelper(specType, specNewStr, fastslow=True, include_heading=True)

            else:
                self.dPatchingExecutor.spec[specType][self.dPatchingExecutor.robotName] = \
                    self.dPatchingExecutor.sendSpecHelper(specType, specStr, fastslow=True, include_heading=True)

        # send also the old sysGoals
        self.dPatchingExecutor.sysGoalsOld[self.dPatchingExecutor.robotName] = \
            self.dPatchingExecutor.sendSpecHelper('SysGoalsOld', mySpec['SysGoals'], fastslow=True, include_heading=True, current_goal_id=int(self.prev_z))

        # send prop
        self.dPatchingExecutor.envPropList[self.dPatchingExecutor.robotName] = \
            self.dPatchingExecutor.sendPropHelper('env', self.sensor_strategy.getInputs(expand_domains=True))


        # TODO: Here we assume we are using bits
        # replace sys init with env init for regions
        sysProps = self.strategy.current_state.getOutputs(expand_domains=True)
        sysProps = {sProp:False if 'region_b' in sProp else sValue for sProp, sValue in sysProps.iteritems()}

        for eProp, eValue in self.sensor_strategy.getInputs(expand_domains=True).iteritems():
            if 'regionCompleted_b' in eProp and eValue:
                sysProps[eProp.replace('regionCompleted_b', 'region_b')] = eValue

        self.dPatchingExecutor.sysPropList[self.dPatchingExecutor.robotName] = self.dPatchingExecutor.sendPropHelper('sys', sysProps)

        # save proposition mapping
        self.dPatchingExecutor.propMappingNewToOld[self.dPatchingExecutor.robotName] = {}
        self.dPatchingExecutor.propMappingOldToNew[self.dPatchingExecutor.robotName] = {}

        for prop in self.dPatchingExecutor.envPropList[self.dPatchingExecutor.robotName].keys() + self.dPatchingExecutor.sysPropList[self.dPatchingExecutor.robotName].keys():
            self.dPatchingExecutor.propMappingNewToOld[self.dPatchingExecutor.robotName].update({prop:prop})
            self.dPatchingExecutor.propMappingOldToNew[self.dPatchingExecutor.robotName].update({prop:prop})
        ltlmop_logger.debug("we should have finished setting up things for ourselves")

    def initiateDPatching(self, received_request=False):
        """
        This function prepares for dencentralized global strategy with the other robots.
        received_request: False if we initiate patching. True if we received request
        """
        # update the other robots with the latest location of us
        self.updateLatestRegionInfoWithAllRobots()

        # also stops the other robots operating
        for robot in self.dPatchingExecutor.robotInRange:
            self.dPatchingExecutor.setPauseForControllerSynthesis(self.dPatchingExecutor.clients[robot], True)

        # then send spec and props to the robot requesting cooridination
        # now only send requests to robots violating the spec
        robotsInConflict = self.checkRobotsInConflict(list(set(self.violated_spec_list + self.possible_states_violated_spec_list)))
        ltlmop_logger.debug("list(set(self.violated_spec_list + self.possible_states_violated_spec_list)):" + str(list(set(self.violated_spec_list + self.possible_states_violated_spec_list))))
        ltlmop_logger.debug("robotsInConflict:" + str(robotsInConflict))
        if robotsInConflict: # list not empty. Some robots is in conflict with us
            #self.dPatchingExecutor.coordinationRequestSent = robotsInConflict
            self.dPatchingExecutor.setCoordinationRequestSent(robotsInConflict)
            self.postEvent("D-PATCH","We will now ask for a centralized strategy be executed.")
            self.runCentralizedStrategy = True
        elif received_request:
            ltlmop_logger.info("We are asked to join patching")
            self.runCentralizedStrategy = True

        else:
            ltlmop_logger.warning("we need to trigger env characterization instead. This is not checked yet!")
            self.postEvent("INFO","Violation is only about our propositions. Carrying out environment characterization")
            ltlmop_logger.debug("list(set(self.violated_spec_list + self.possible_states_violated_spec_list)):" + str(list(set(self.violated_spec_list + self.possible_states_violated_spec_list))))
            self.addStatetoEnvSafety(self.sensor_strategy)

        if robotsInConflict or received_request:
            # send spec to request received and also violated robots
            csockRequestReceived = [self.dPatchingExecutor.clients[x] for x in [robot for robot, status in self.dPatchingExecutor.coordinationRequest.iteritems() if status]]
            csockRobotsInConflict = [self.dPatchingExecutor.clients[robot] for robot in robotsInConflict]
            self.prepareForCentralizedStrategySnippetToOtherRobots(list(set(csockRobotsInConflict) | set(csockRequestReceived)))
            #self.prepareForCentralizedStrategySnippetToOtherRobots(list(set(self.dPatchingExecutor.clients.values()) - set([self.dPatchingExecutor.serv])))

            # first put together our spec
            self.prepareForCentralizedStrategySnippetToSelf()
            # taking care all the synthesis and wait till everyone is ready.
            self.synthesizeGlobalStrategyAndWaitForOthersToResume()

            # set up global envTrans check
            self.setupGlobalEnvTransCheck()

        # update locations and sensors once more
        self.updateLatestRegionInfoWithAllRobots()

        # allows the other robots to move again
        for robot in self.dPatchingExecutor.robotInRange:
            self.dPatchingExecutor.setPauseForControllerSynthesis(self.dPatchingExecutor.clients[robot], False)

    def updateLatestRegionInfoWithAllRobots(self):
        """
        This function updates all region info (including heading if necessary to all robots)
        """
        # update the other robots with the latest location of us
        if self.proj.compile_options['include_heading']:
            # update region info for all connected robots.
            self.dPatchingExecutor.updateRobotRegionWithAllClients(self.strategy.current_state.getPropValue('region'))
            self.dPatchingExecutor.updateCompletedRobotRegionWithAllClients(self.sensor_strategy.getPropValue('regionCompleted'))
        else:
            self.dPatchingExecutor.updateRobotRegionWithAllClients(self.sensor_strategy.getPropValue('regionCompleted'))

        # update sensor info too
        enabled_sensors = [x for x in self.proj.enabled_sensors if not (x.endswith('_rc') or x.startswith(tuple(self.dPatchingExecutor.robotInRange)))]
        ltlmop_logger.warning('self.hsub.getSensorValue(enabled_sensors):' + str(self.hsub.getSensorValue(enabled_sensors)))
        self.dPatchingExecutor.updateRobotSensorsWithAllClients(self.hsub.getSensorValue(enabled_sensors))

    def setupGlobalEnvTransCheck(self):
        """
        This function setup the checkviolation object with the global envTrans used in the global strategy
        """
        # set up global envTrans check
        #globalEnvTrans = " &\n".join(filter(None, [self.dPatchingExecutor.spec['EnvTrans'][robot] for robot in self.dPatchingExecutor.coordinatingRobots])) # OLD
        # (also filter coordinating robot EnvTrans)
        globalEnvTrans = " &\n".join(filter(None, [self.dPatchingExecutor.filterAndExcludeSpecOfCoordinatingRobots(self.dPatchingExecutor.spec['EnvTrans'][robot], robot)\
                                                     for robot in self.dPatchingExecutor.coordinatingRobots]))

        ltlmop_logger.debug('globalEnvTrans:' + str(globalEnvTrans))
        # globalEnvTransList = []
        # for robot in self.dPatchingExecutor.coordinatingRobots:
        #     globalEnvTransList.append(self.dPatchingExecutor.filterAndExcludeSpecOfCoordinatingRobots(self.spec['EnvTrans'][robot], robot))
        # globalEnvTrans = " &\n".join(filter(None, ))

        self.globalEnvTransCheck = LTLParser.LTLcheck.LTL_Check(None, {}, {'EnvTrans': globalEnvTrans}, 'EnvTrans')
        self.globalEnvTransCheck.ltl_treeEnvTrans = LTLParser.LTLFormula.parseLTL(globalEnvTrans)
        ltlmop_logger.debug('finished setupGlobalEnvTransCheck')
    def checkRobotsInConflict(self, violatedEnvTransList):
        """
        This function checks the robots involved in the conflict.
        There could be no robots but just abnormal sensors.
        return list of robots in conflict, not including myself.
        """
        robotNameCoordinating = []
        for ltlSpec in violatedEnvTransList: # list
            for robot in self.dPatchingExecutor.robotInRange + [self.dPatchingExecutor.robotName]:
                robotInFormula = LTLParser.LTLcheck.checkIfKeyInFormula(ltlSpec, robot)

                if robotInFormula and robot not in robotNameCoordinating:
                    robotNameCoordinating.append(robot)

        return robotNameCoordinating

    def filterAndExcludeSpecOfCoordinatingRobots(self, violatedList, specStr):
        """
        This function takes in the violated envTrans, first figure out the robots in conflict, then remove spec relating the robots from specStr if the specStr contains it.
        violatedList: violated EnvTrans list
        specStr: EnvTrans spec string
        """
        # first find out robots coordinating
        robotsInConflict = self.checkRobotsInConflict(violatedList)

        #then send in the list of coordinating robots to remove spec (need to pass in str list)
        specStrList = LTLParser.LTLcheck.ltlStrToList(specStr)

        coordinatingRobots = list(set(robotsInConflict + [self.dPatchingExecutor.robotName]  + [k for k, v in self.dPatchingExecutor.coordinationRequest.iteritems() if v]))
        # keylist = all robots, keymatch = conflicting robots
        specFilteredList = LTLParser.LTLcheck.filterRelatedRobotSpec(specStrList, \
            self.dPatchingExecutor.robotInRange + [self.dPatchingExecutor.robotName],
            coordinatingRobots, self.dPatchingExecutor.robotName)


        #specNewStr = LTLParser.LTLcheck.excludeSpecFromFormula(specStr, specToExclude)
        specNewStr = "&\n".join(filter(None, specFilteredList))

        return specNewStr

    def filterAndExcludeSpec(self, violatedList, specStr):
        """
        This function takes in the violated envTrans, first filter those with only one robot, then remove in from specStr if the specStr contains it.
        violatedList: violated EnvTrans list
        specStr: EnvTrans spec string
        """
        ltlmop_logger.debug("violatedList:" + str(violatedList))
        violatedListFiltered = LTLParser.LTLcheck.filterOneRobotViolatedSpec(violatedList, self.dPatchingExecutor.robotInRange + [self.dPatchingExecutor.robotName])
        ltlmop_logger.debug("violatedListFiltered:" + str(violatedListFiltered))

        specNewStr = LTLParser.LTLcheck.excludeSpecFromFormula(specStr, violatedListFiltered)
        #specNewStr = "&\n".join(filter(None,violatedListFiltered))
        ltlmop_logger.debug("specNewStr:" + specNewStr)
        return specNewStr

    def synthesizeGlobalStrategyAndWaitForOthersToResume(self):
        """
        This function asks the dPatchingExecutor to synthesize global strategy.
        After that it waits till all coorindating robots are ready to restart.
        """
        # then wait till all surrounding robots are ready as well
        while not self.dPatchingExecutor.prepareForCentralizedExecution():
            ltlmop_logger.warning('we are still waiting parts from the other robots')

            self.dPatchingExecutor.runIterationNotCentralExecution()
            ltlmop_logger.debug("self.dPatchingExecutor.coordinatingRobots:" + str(self.dPatchingExecutor.coordinatingRobots))
            time.sleep(0.2)

        # now wait till the other robot has synthesized an automaton
        # TODO: what if it's unrealizable? We should know that as well.
        while not self.dPatchingExecutor.checkIfOtherRobotsAreReadyToExecuteCentralizedStrategy():
            ltlmop_logger.warning('we are still waiting completion of synthesis from the other robots')

            self.dPatchingExecutor.runIterationNotCentralExecution()
            time.sleep(0.2)

    def initiateDPatchingCentralizedMode(self, received_request=False):
        """
        This function is to trigger coordination when we are already coordinating.
        received_request: False if we initiate patching. True if we received request
        """
        # update the other robots with the latest location of us
        self.updateLatestRegionInfoWithAllRobots()

        # also stops the other robots operating
        for robot in self.dPatchingExecutor.robotInRange:
            self.dPatchingExecutor.setPauseForControllerSynthesis(self.dPatchingExecutor.clients[robot], True)

        # then send spec and props to the robot requesting cooridination
        # now only send requests to robots violating the spec
        robotsInConflict = self.checkRobotsInConflict(list(set(self.violated_spec_list + self.possible_states_violated_spec_list)))
        if robotsInConflict: # list not empty. Some robots is in conflict with us
            ################################
            ########## PREPARATION #########
            ################################
            # save old copy of coordinating robots
            self.dPatchingExecutor.old_coordinatingRobots = copy.deepcopy(self.dPatchingExecutor.coordinatingRobots)

            # remove violated envTrans from list.
            #for robot in self.dPatchingExecutor.spec['EnvTrans'].keys():
            #    ltlmop_logger.debug("Robot Under consideration:" + str(robot))
            #    self.dPatchingExecutor.spec['EnvTrans'][robot] = self.filterAndExcludeSpec(list(set(self.violated_spec_list + self.possible_states_violated_spec_list)), self.dPatchingExecutor.spec['EnvTrans'][robot])
            #    self.setupGlobalEnvTransCheck() # update EnvTrans globel by setting up a new object
            #    ltlmop_logger.debug("-------------------------------------------")

            # add violations of local spec to LTL violated_str list
            for specStr in list(set(self.violated_spec_list + self.possible_states_violated_spec_list)):
                # replace e.g. alice_r1_rc to alice_r1 as the meaning changes in the central strategy
                for region in self.dPatchingExecutor.regionList:
                    for otherRobot in list(set(self.dPatchingExecutor.coordinationRequestSent) | set([robot for robot in self.dPatchingExecutor.coordinationRequest.keys()])):
                        specStr = re.sub('(?<=[! &|(\t\n])e.'+otherRobot+'_'+region+'_rc'+'(?=[ &|)\t\n])', 'e.'+otherRobot+'_'+region, specStr)
                self.LTLViolationCheck.violated_specStr.append(specStr)
                self.LTLViolationCheckPossibleStates.violated_specStr.append(specStr)

            ltlmop_logger.debug("self.LTLViolationCheck.violated_specStr (also LTLViolationCheckPossibleStates):" + str(self.LTLViolationCheck.violated_specStr))

            #################################################
            #### CHECK IF NEW ROBOTS ARE INVOLVED #########
            #################################################
            # first checks if we are already coorindating with the robots in conflict
            if set(robotsInConflict).issubset(self.dPatchingExecutor.coordinatingRobots):
                # reset coordination status
                self.dPatchingExecutor.resetRobotStatusOnCentralizedStrategyExecution()

                # all the robots in conflicts are already coorindating. Just remove violated line and resynthesize.
                ltlmop_logger.warning("in conflict robots are the same. just remove envTrans and resynthesize")
                self.postEvent("RESOLVED","Current coordinating robots only. Removing violated environment assumptions and resynthesize.")

            else:
                # some new robots are joining, we should ask them to join centralized strategy.
                ltlmop_logger.warning("invite other robots to join the centralized execution")

                # send spec to request received and also violated robots
                csockNewRobotsToCoordinate = [self.dPatchingExecutor.clients[robot] for robot in robotsInConflict if not robot in self.dPatchingExecutor.old_coordinatingRobots]

                #only send local spec here.(Assuming the other robot coordinating find out the same problem and send its own)
                self.prepareForCentralizedStrategySnippetToOtherRobots(csockNewRobotsToCoordinate)

                #update coordination sent
                #self.dPatchingExecutor.coordinationRequestSent = robotsInConflict
                self.dPatchingExecutor.setCoordinationRequestSent(robotsInConflict)

        elif received_request:
            ltlmop_logger.info("We are asked to coordinate with more robots!")
        else:
            self.postEvent("INFO","Violation is only about our propositions. Carrying out environment characterization")
            self.addStatetoEnvSafety(self.dPatchingExecutor.sensor_state, checker=self.globalEnvTransCheck)
            ltlmop_logger.warning("we need to trigger env characterization instead. This is not checked!")

        if robotsInConflict or received_request:
            # make sure all sensors are the latest
            for prop_name, value in self.dPatchingExecutor.convertFromRegionBitsToRegionNameInDict('env', self.sensor_strategy.getInputs(expand_domains = True)).iteritems():
                self.dPatchingExecutor.sensor_state.setPropValue(self.dPatchingExecutor.propMappingOldToNew[self.dPatchingExecutor.robotName][prop_name], value)

            # taking care all the synthesis and wait till everyone is ready.
            self.synthesizeGlobalStrategyAndWaitForOthersToResume()

            # set up global envTrans check
            self.setupGlobalEnvTransCheck()

        # update locations and sensors once more
        self.updateLatestRegionInfoWithAllRobots()

        # also stops the other robots operating
        for robot in self.dPatchingExecutor.robotInRange:
            self.dPatchingExecutor.setPauseForControllerSynthesis(self.dPatchingExecutor.clients[robot], False)
    # %%%%%%%%%%%%%%%%%%%%%%%%%%% #
