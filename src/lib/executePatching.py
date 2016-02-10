import os
import re
import itertools
import specCompiler
import time, math
import numpy

import parseEnglishToLTL
import LTLParser.translateFromSlugsLTLFormatToLTLFormat
import sets # make sure there's no duplicates of elements
import LTLParser.LTLRegion # for adding region parentheses

# logger for ltlmop
import logging
ltlmop_logger = logging.getLogger('ltlmop_logger')
import copy

class ExecutorPatchingExtensions(object):
    """ Extensions to Executor to allow for glocal patching. """

    def __init__(self):
        super(ExecutorPatchingExtensions, self).__init__()
        ltlmop_logger.info("Initializing patching extensions...")

    def checkEnvTransViolationWithNextPossibleStates(self, checker=None, current_state=None, sensor_state=None):
        """
        This function obtains all possible next states and check if they will violate the environment assumptions.
        """
        # set to default if no value is given
        if checker is None:
            checker =self.LTLViolationCheck
        if current_state is None:
            current_state=self.strategy.current_state
        if sensor_state is None:
            sensor_state=self.sensor_strategy

        deepcopy_current_state = copy.deepcopy(current_state)
        deepcopy_sensor_state = copy.deepcopy(sensor_state)

        if self.proj.compile_options["multi_robot_mode"] == "patching":
            otherEnvPropDict = self.robClient.requestNextPossibleEnvStatesFromOtherRobot()
            robotNameList = [self.robClient.robotName]
        elif self.proj.compile_options["multi_robot_mode"] == "d-patching":
            otherEnvPropDict = self.dPatchingExecutor.getNextPossibleEnvStatesFromOtherRobots()
            robotNameList = list(set([self.dPatchingExecutor.robotName] + self.dPatchingExecutor.coordinatingRobots))
        else:
            ltlmop_logger.warning('not in a valid mode - ' + str(self.proj.compile_options["multi_robot_mode"]))

        # first verify that we won't be skipping th check
        if list(set(otherEnvPropDict.keys()+[self.dPatchingExecutor.robotName])) != robotNameList:
            ltlmop_logger.debug("we are checking propCombination")
            for propCombination in itertools.product(*[v for k,v in otherEnvPropDict.iteritems() if k not in robotNameList]):
                #ltlmop_logger.debug("propCombination:" + str(propCombination))
                # assignments to sensor_strategy for each combination
                for propDict in propCombination:
                    for propKey, propValue in propDict.iteritems():
                        # check if key exist in sensor strategy?
                        if propKey in deepcopy_sensor_state.getInputs(expand_domains=True).keys():
                            deepcopy_sensor_state.setPropValue(propKey, propValue)

                    # the current state stays the same but checks with differnt next possible states
                    env_assumption_hold = checker.checkViolation(deepcopy_current_state, deepcopy_sensor_state)
                    if not env_assumption_hold:
                        ltlmop_logger.debug("POSSIBLE STATES - asssumptions violated!")
                        ltlmop_logger.debug("current_state:" + str([k for k, v in deepcopy_current_state.getAll(expand_domains=True).iteritems() if v]))
                        ltlmop_logger.debug("sensor_state" + str([k for k, v in deepcopy_sensor_state.getInputs(expand_domains=True).iteritems() if v]))
                        ltlmop_logger.debug("checker.violated_specStr:" + str(checker.violated_specStr))
                        return False
        else:
            ltlmop_logger.debug("we have all the robots. doing only once")
            env_assumption_hold = checker.checkViolation(deepcopy_current_state, deepcopy_sensor_state)
            if not env_assumption_hold:
                ltlmop_logger.debug("POSSIBLE STATES - asssumptions violated!")
                ltlmop_logger.debug("current_state:" + str([k for k, v in deepcopy_current_state.getAll(expand_domains=True).iteritems() if v]))
                ltlmop_logger.debug("sensor_state" + str([k for k, v in deepcopy_sensor_state.getInputs(expand_domains=True).iteritems() if v]))
                ltlmop_logger.debug("checker.violated_specStr:" + str(checker.violated_specStr))
                return False
        return True

    def loadSpecObjectFromFile(self, spec_file = ""):
        """
        spec_file: path to specification
        """

        if not spec_file:
            spec_file = os.path.join(self.proj.project_root, self.proj.getFilenamePrefix() + ".spec")

        compiler = specCompiler.SpecCompiler(spec_file)
        compiler._decompose()
        spec, tracebackTree, response = compiler._writeLTLFile()
        # replace EnvTrans check to make sure it's not empty
        spec['EnvTrans'] = '&\n '.join(filter(None, [spec['EnvTrans'], spec["EnvTopo"]]))

        # replace EnvInit and SysInit to just init topology for centralized synthesis
        spec['EnvInit'] = spec['InitEnvRegionSanityCheck']
        spec['SysInit'] = spec['InitRegionSanityCheck']

        # replace SysTrans
        spec['SysTrans'] = '&\n '.join(filter(None, [spec['SysTrans'], spec["Topo"]]))

        # replace EnvGoals
        spec['EnvGoals'] = '&\n '.join(filter(None, [spec['EnvGoals'], spec["SysImplyEnv"]]))

        return spec

    def prepareForCentralizedStrategy(self):
        """
        This function prepares for the synthesis of centralized strategy.
        (Same as testRobClient.py)
        """

        # load spec
        mySpec = self.loadSpecObjectFromFile()

        # send spec
        for specType, specStr in mySpec.iteritems():
            if specType not in ['SysInit','SysTrans','SysGoals','EnvInit','EnvTrans','EnvGoals']:
                continue

            # tell robClient the current goal we are pursuing
            if specType == 'SysGoals':
                ltlmop_logger.warning("SLUGS does not output bddLivenesses now and this will fail.")
                specNewStr = self.extractCurrentLivenessWithWinningPositions(self.strategy.current_state.goal_id) #FIXIT: no bddLivenesses now.
                self.robClient.sendSpec(specType, specNewStr, fastslow=True, include_heading=True)
                #self.robClient.sendSpec(specType, specStr, fastslow=True, include_heading=True, current_goal_id=int(self.strategy.current_state.goal_id))

                # print current goal pursuing
                #currentGoalLTL = (str(LTLParser.LTLcheck.ltlStrToList(specStr)[int(self.strategy.current_state.goal_id)]) if not specStr.count('[]<>') == 1 else specStr)
                self.postEvent("PATCH", "Current goal is " + self.proj.specText.split('\n')[self.tracebackTree['SysGoals'][int(self.strategy.current_state.goal_id)]-1])

            else:
                self.robClient.sendSpec(specType, specStr, fastslow=True, include_heading=True)

        # send also the old sysGoals
        self.robClient.sendSpec('SysGoalsOld', mySpec['SysGoals'], fastslow=True, include_heading=True,current_goal_id=int(self.strategy.current_state.goal_id))

        # send prop
        self.robClient.sendProp('env', self.sensor_strategy.getInputs(expand_domains = True))
        #self.robClient.sendProp('env', self.strategy.current_state.getInputs(expand_domains = True))
        ltlmop_logger.debug("Using sensor state. Should be the violation one" + str(self.sensor_strategy.getInputs()))
        # TODO: Here we assume we are using bits
        # replace sys init with env init for regions
        sysProps = self.strategy.current_state.getOutputs(expand_domains = True)
        sysProps = {sProp:False if 'region_b' in sProp else sValue for sProp,sValue in sysProps.iteritems() }

        for eProp, eValue in self.sensor_strategy.getInputs(expand_domains = True).iteritems():
            if 'regionCompleted_b' in eProp and eValue:
                sysProps[eProp.replace('regionCompleted_b','region_b')] = eValue

        self.robClient.sendProp('sys', sysProps)

        # send cooridination status
        self.robClient.setCoordinationStatus(True)

    def initiatePatching(self):
        """
        This function takes care of the procedure for switching from individual strategy to centralized strategy
        """
        # first prepare for centralized execution
        self.prepareForCentralizedStrategy()

        # then wait till all surrounding robots are ready as well
        while not self.robClient.getCentralizedExecutionStatus():
            time.sleep(2)

    def extractCurrentLivenessWithWinningPositions(self,sysGoalsId):
        """
        This function obtains the conjunction of the current liveness with winning positions.
        sysGoalsId: current system goal number
        """

        # first read sysGoals LTL with winning positions preComputed.
        if self.proj.compile_options['symbolic']:
            with open(self.proj.getFilenamePrefix()+".bddliveness"+ str(sysGoalsId), 'r') as f:
                slugsStr = f.read()
        else:
            with open(self.proj.getFilenamePrefix()+".autliveness"+ str(sysGoalsId), 'r') as f:
                slugsStr = f.read()
        f.closed

        # modify sensor list to parse region props properly
        sensorList = self.proj.enabled_sensors
        if self.proj.compile_options['fastslow']:
            # remove _rc props from sensor_list
            sensorList = [x for x in sensorList if not x.endswith('_rc') or x.startswith(tuple(self.proj.otherRobot))]
            if self.proj.compile_options["use_region_bit_encoding"]:
                #add in sbit for region completion
                sensorList.extend(["sbit"+str(i) for i in range(0,int(numpy.ceil(numpy.log2(len(self.proj.rfi.regions)))))])
            else:
                # added in region_rc with the decomposed region names
                sensorList.extend([r.name+"_rc" for r in self.proj.rfi.regions])

        #ltlmop_logger.debug("sensorList:" + str(sensorList))
        # convert formula from slugs to our format
        sysGoalsLTLList = LTLParser.translateFromSlugsLTLFormatToLTLFormat.parseSLUGSCNFtoLTLList(slugsStr,sensorList)
        #ltlmop_logger.debug(sysGoalsLTLList)

        # replace with normal region bits (like actual regions)
        sysGoalsLTL = self.replaceIndividualSbitsToGroupSbits(sysGoalsLTLList)
        #ltlmop_logger.debug(sysGoalsLTL)

        return "[]<>("+sysGoalsLTL+")"


    def extractWinningPositions(self):
        """
        This function obtains the conjunction of the current liveness with winning positions.
        sysGoalsId: current system goal number
        """
        # first read sysGoals LTL with winning positions preComputed.
        if self.proj.compile_options['symbolic']:
            with open(self.proj.getFilenamePrefix()+".bddWinPos", 'r') as f:
                slugsStr = f.read()
        else:
            with open(self.proj.getFilenamePrefix()+".autWinPos", 'r') as f:
                slugsStr = f.read()
        f.closed

        # modify sensor list to parse region props properly
        sensorList = self.proj.enabled_sensors
        if self.proj.compile_options['fastslow']:
            # remove _rc props from sensor_list
            sensorList = [x for x in sensorList if not x.endswith('_rc') or x.startswith(tuple(self.proj.otherRobot))]
            if self.proj.compile_options["use_region_bit_encoding"]:
                #add in sbit for region completion
                sensorList.extend(["sbit"+str(i) for i in range(0,int(numpy.ceil(numpy.log2(len(self.proj.rfi.regions)))))])
            else:
                # added in region_rc with the decomposed region names
                sensorList.extend([r.name+"_rc" for r in self.proj.rfi.regions])

        #ltlmop_logger.debug("sensorList:" + str(sensorList))
        # convert formula from slugs to our format
        sysGoalsLTLList = LTLParser.translateFromSlugsLTLFormatToLTLFormat.parseSLUGSCNFtoLTLList(slugsStr,sensorList)

        # replace with normal region bits (like actual regions)
        sysWinningPosLTL = self.replaceIndividualSbitsToGroupSbits(sysGoalsLTLList)

        return "("+sysWinningPosLTL+")"

    def replaceIndividualSbitsToGroupSbits(self, LTLList):
        """
        This function takes in an LTL list, extract sbits and put in the right bits afterwards
        """
        numBits = int(math.ceil(math.log(len(self.proj.rfi.regions),2)))
        # TODO: only calc bitencoding once
        bitEncode = parseEnglishToLTL.bitEncoding(len(self.proj.rfi.regions), numBits)
        currBitEnc = bitEncode['current']
        nextBitEnc = bitEncode['next']
        envBitEnc = bitEncode['env']
        envNextBitEnc = bitEncode['envNext']

        newLTLList = sets.Set([])
        #newLTLList = []

        for LTLStr in LTLList: # LTLStr are of the form  a | b | c
            propList = sets.Set(LTLStr.split(' | '))
            # run four times
            propList = self.replaceIndividualSbitsToGroupSbitsForTypeOfBits(numBits, '!?next\(s.bit(?P<bitNo>\d+)\)', nextBitEnc, propList)
            propList = self.replaceIndividualSbitsToGroupSbitsForTypeOfBits(numBits, '!?s.bit(?P<bitNo>\d+)', currBitEnc, propList)
            propList = self.replaceIndividualSbitsToGroupSbitsForTypeOfBits(numBits, '!?next\(e.sbit(?P<bitNo>\d+)\)', envNextBitEnc, propList)
            propList = self.replaceIndividualSbitsToGroupSbitsForTypeOfBits(numBits, '!?e.sbit(?P<bitNo>\d+)', envBitEnc, propList)

            #newLTLList.append(' | '.join(propList))
            newLTLList.add(propList)

        #return '&\n'.join(['('+x+')' for x in newLTLList])
        return '&\n'.join(['('+' | '.join(x)+')' for x in newLTLList])


    def replaceIndividualSbitsToGroupSbitsForTypeOfBits(self, numBits, strPattern, bitsList, propList):
        """
        This function is one iteration to replace bits
        numBits: no. of bits in consideration
        strPattern: pattern to match
        bitsList: one of the lists in the bitEncoding
        propList = set of props
        """
        regionPropList = []
        validRegionPropList = sets.Set([])
        # first extract all ebits
        for prop in propList:
            if re.match(strPattern, prop) is not None:
                regionPropList.append(prop)

        for regionProp in regionPropList:
            bitInConsideration = int(re.match(strPattern,regionProp).group('bitNo'))
            numBitsToEnumerate = numBits-1
            bitStrList = []
            # if we have three bits, then only enumerate 2
            if numBitsToEnumerate:
                for x in range(2**numBitsToEnumerate):
                    # also reverse bit so that we have bits 0 1 2 ...
                    tempBitStr = "{0:0{width}b}".format(x, width=numBitsToEnumerate)
                    # add the missing bit at the right position
                    bitStrList.append(tempBitStr[:bitInConsideration] + ('0' if '!' in regionProp else'1') + tempBitStr[bitInConsideration:])
            else:
                # only one bit
                bitStrList.append(('0' if '!' in regionProp else'1'))

            # find all region bit str now
            for regionIdx in [int(x,2) for x in bitStrList]:
                if regionIdx < len(self.proj.rfi.regions):
                    validRegionPropList.add(bitsList[regionIdx])

            # remove regionProp from propList and append validRegionPropList to propList
            propList.remove(regionProp)
            propList.update(validRegionPropList)

        return propList
