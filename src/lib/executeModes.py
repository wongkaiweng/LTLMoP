import time
import copy
import LTLParser.LTLcheck

# logger for ltlmop
import logging
ltlmop_logger = logging.getLogger('ltlmop_logger')

class ExecutorModesExtensions(object):
    """ Extensions to Executor to allow for specification rewriting and resynthesis.
        This class is not meant to be instantiated. """

    def __init__(self):
        super(ExecutorModesExtensions, self).__init__()
        ltlmop_logger.info("Initializing modes extensions...")

        self.env_assumption_hold = True # keeping track if env trans holds with/without possible next states.
        self.violated_spec_list = [] # keep track of specs violated
        self.violated_spec_list_with_no_specText_match = [] # keep track of specs violated but with no specText match
        self.violated_spec_line_no = [] # keep track of specText lines violated.
        #self.violation_resolved = True # make sure violation holds until they are resolved.

        self.possible_states_env_assumption_hold = True # keeping track if env trans holds with/without possible next states.
        self.possible_states_violated_spec_list = [] # keep track of specs violated
        self.possible_states_violated_spec_list_with_no_specText_match = [] # keep track of specs violated but with no specText match
        self.possible_states_violated_spec_line_no = [] # keep track of specText lines violated.

    def simple_check_envTrans_violation(self):
        """
        This version of check violation only monitor the current sensors and incoming sensors.
        """
        # Take a snapshot of our current sensor readings
        sensor_state = copy.deepcopy(self.hsub.getSensorValue(self.proj.enabled_sensors))
        for prop_name, value in sensor_state.iteritems():
            if self.proj.compile_options['fastslow'] and prop_name.endswith('_rc') and not prop_name.startswith(tuple(self.proj.otherRobot)):
                continue

            self.sensor_strategy.setPropValue(prop_name, value)

        if self.proj.rfi:
            if self.proj.compile_options['fastslow']:
            #    curRegionIdx = self._getCurrentRegionFromPose()
            #    if curRegionIdx is None:
            #        curRegionIdx = self.proj.rfi.indexOfRegionWithName(decomposed_region_names[0])
                currentRegionCompleted_oldName = [prop_name for prop_name, value in sensor_state.iteritems() \
                    if prop_name.endswith('_rc') and not prop_name.startswith(tuple(self.proj.otherRobot)) and value]

                # check we are using decomposition. does it differently
                if self.proj.compile_options['decompose']:
                    currentRegionCompleted_newName = self.proj.regionMapping[currentRegionCompleted_oldName[0].replace('_rc','')]
                    curRegionIdx = self.proj.rfi.indexOfRegionWithName(currentRegionCompleted_newName[0])
                else: # old name is sufficient
                    curRegionIdx = self.proj.rfi.indexOfRegionWithName(currentRegionCompleted_oldName[0].replace('_rc',''))

                self.sensor_strategy.setPropValue("regionCompleted", self.proj.rfi.regions[curRegionIdx])

        deepcopy_sensor_state = copy.deepcopy(self.sensor_strategy)
        deepcopy_current_state = copy.deepcopy(self.strategy.current_state)

        if self.proj.compile_options['neighbour_robot'] and self.proj.compile_options["multi_robot_mode"] == "patching":
            # ************ patching ****************** #
            env_assumption_hold, self.violated_spec_line_no, self.violated_spec_list, self.violated_spec_list_with_no_specText_match = \
                self.LTLViolationCheck.checkViolationWithListReturned(deepcopy_current_state, deepcopy_sensor_state)
            # **************************************** #

        elif self.proj.compile_options['neighbour_robot'] and self.proj.compile_options["multi_robot_mode"] == "d-patching":
            # %%%%%%%%%%%%% d-patching %%%%%%%%%%%%%%% #
            if self.runCentralizedStrategy:
                for prop_name, value in self.dPatchingExecutor.convertFromRegionBitsToRegionNameInDict('env', self.sensor_strategy.getInputs(expand_domains=True)).iteritems():
                    self.dPatchingExecutor.sensor_state.setPropValue(self.dPatchingExecutor.propMappingOldToNew[self.dPatchingExecutor.robotName][prop_name], value)

                # now updates robot sensors
                #ltlmop_logger.warning("self.robotSensors:" + str(self.robotSensors))
                for coR in self.dPatchingExecutor.coordinatingRobots:
                    if coR != self.dPatchingExecutor.robotName:
                        nextRobotSensors = {self.dPatchingExecutor.propMappingOldToNew[coR][k]:v for k, v in self.dPatchingExecutor.robotSensors[coR].iteritems()}
                        #ltlmop_logger.warning('nextRobotSensors:' + str(nextRobotSensors))
                        self.dPatchingExecutor.sensor_state.setPropValues(nextRobotSensors)

                deepcopy_sensor_state = copy.deepcopy(self.dPatchingExecutor.sensor_state)
                deepcopy_current_state = copy.deepcopy(self.dPatchingExecutor.strategy.current_state)
                env_assumption_hold, self.violated_spec_line_no, self.violated_spec_list, self.violated_spec_list_with_no_specText_match = \
                    self.globalEnvTransCheck.checkViolationWithListReturned(deepcopy_current_state, deepcopy_sensor_state)

                if not env_assumption_hold:
                    # now checks if it's only about one coordinating robot
                    for x in self.globalEnvTransCheck.violated_specStr:
                        if LTLParser.LTLcheck.filterSpecList([x], self.dPatchingExecutor.robotInRange + [self.dPatchingExecutor.robotName]):
                            break
                    else:
                        env_assumption_hold = True
                        ltlmop_logger.debug("no violations as it's only about one robot (later should change to only topology)")
            else:
                env_assumption_hold, self.violated_spec_line_no, self.violated_spec_list, self.violated_spec_list_with_no_specText_match = \
                    self.LTLViolationCheck.checkViolationWithListReturned(deepcopy_current_state, deepcopy_sensor_state)
            #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% #
        else:
            # Check for environment violation - change the env_assumption_hold to int again
            env_assumption_hold, self.violated_spec_line_no, self.violated_spec_list, self.violated_spec_list_with_no_specText_match = \
                self.LTLViolationCheck.checkViolationWithListReturned(deepcopy_current_state, deepcopy_sensor_state)

        if not env_assumption_hold:
            ltlmop_logger.debug("======== envTrans violations ============")
            ltlmop_logger.debug("ACTUAL-current_state:" + str([x for x, value in deepcopy_current_state.getAll(expand_domains=True).iteritems() if value]))
            ltlmop_logger.debug("ACTUAL-sensor_state:" + str([x for x, value in deepcopy_sensor_state.getInputs(expand_domains=True).iteritems() if value]))
            ltlmop_logger.debug("ACTUAL-env_assumption_hold:" + str(env_assumption_hold))

        return env_assumption_hold

    def run_simple_check_envTrans_violations(self):
        """
        This function puts checkEnvTransViolationWithNextPossibleStates into a while loop for using with thread
        """
        sensor_state = None
        otherEnvPropDict = None
        current_state = None

        old_sensor_state = None
        old_otherEnvPropDict = None
        old_current_state = None
        try:
            while self.alive.isSet():
                if self.runStrategy.isSet() and self.runRuntimeMonitoring.isSet():
                    # only run check violation if sensor or possible next states has changed
                    runCheck = False
                    sensor_state = copy.deepcopy(self.hsub.getSensorValue(self.proj.enabled_sensors))
                    #if self.proj.compile_options['neighbour_robot'] and self.proj.compile_options["multi_robot_mode"] == "patching":
                    #    otherEnvPropDict = self.robClient.requestNextPossibleEnvStatesFromOtherRobot()
                    #elif self.proj.compile_options['neighbour_robot'] and self.proj.compile_options["multi_robot_mode"] == "d-patching":
                    #    otherEnvPropDict = self.dPatchingExecutor.getNextPossibleEnvStatesFromOtherRobots()

                    # figure out the form of current strategy
                    if self.runCentralizedStrategy:
                        current_state = copy.deepcopy(self.dPatchingExecutor.strategy.current_state)
                    else:
                        current_state = copy.deepcopy(self.strategy.current_state)

                    if (old_current_state is None or old_current_state.getAll(expand_domains=True) != current_state.getAll(expand_domains=True)) or\
                        old_sensor_state != sensor_state: #or old_otherEnvPropDict != otherEnvPropDict:
                        runCheck = True

                    if runCheck and self.runStrategy.isSet() and self.runRuntimeMonitoring.isSet():
                        self.env_assumption_hold = self.simple_check_envTrans_violation()
                        # stop check violations until this one is solved.
                        if not self.env_assumption_hold:
                            self.runRuntimeMonitoring.clear()

                        #if self.runCentralizedStrategy:
                        #    self.violated_spec_list = copy.deepcopy(self.globalEnvTransCheck.violated_specStr)
                        #    self.violated_spec_list_with_no_specText_match = copy.deepcopy(self.globalEnvTransCheck.violated_specStr_with_no_specText_match)
                        #    self.violated_spec_line_no = copy.deepcopy(self.globalEnvTransCheck.violated_spec_line_no)
                        #else:
                        #    self.violated_spec_list = copy.deepcopy(self.LTLViolationCheck.violated_specStr)
                        #    self.violated_spec_list_with_no_specText_match = copy.deepcopy(self.LTLViolationCheck.violated_specStr_with_no_specText_match)
                        #    self.violated_spec_line_no = copy.deepcopy(self.LTLViolationCheck.violated_spec_line_no)

                        if not self.env_assumption_hold:
                            ltlmop_logger.debug("**self.violated_spec_list:" + str(self.violated_spec_list))
                            ltlmop_logger.debug("**self.violated_spec_list_with_no_specText_match:" + str(self.violated_spec_list_with_no_specText_match))
                            ltlmop_logger.debug("**self.violated_spec_line_no:" + str(self.violated_spec_line_no))
                            ltlmop_logger.debug("**self.env_assumption_hold:" + str(self.env_assumption_hold))

                    old_sensor_state = copy.deepcopy(sensor_state)
                    old_otherEnvPropDict = copy.deepcopy(otherEnvPropDict)
                    old_current_state = copy.deepcopy(current_state)
                    #ltlmop_logger.debug("seemed like we did not exit")
        except:
            import traceback
            ltlmop_logger.error(traceback.format_exc())

    def check_envTrans_violations(self):
        """
        This function checks violations of different modes.
        """
       # first stops the robot
        #self.hsub.setTempZeroVelocity()
        #startTime = time.time()

        # Take a snapshot of our current sensor readings
        sensor_state = self.hsub.getSensorValue(self.proj.enabled_sensors)
        for prop_name, value in sensor_state.iteritems():
            if self.proj.compile_options['fastslow'] and prop_name.endswith('_rc') and not prop_name.startswith(tuple(self.proj.otherRobot)):
                continue

            self.sensor_strategy.setPropValue(prop_name, value)

        if self.proj.compile_options['fastslow']:
        #    curRegionIdx = self._getCurrentRegionFromPose()
        #    if curRegionIdx is None:
        #        curRegionIdx = self.proj.rfi.indexOfRegionWithName(decomposed_region_names[0])
            currentRegionCompleted_oldName = [prop_name for prop_name, value in sensor_state.iteritems() \
                if prop_name.endswith('_rc') and not prop_name.startswith(tuple(self.proj.otherRobot)) and value]
            currentRegionCompleted_newName = self.proj.regionMapping[currentRegionCompleted_oldName[0].replace('_rc','')]
            curRegionIdx = self.proj.rfi.indexOfRegionWithName(currentRegionCompleted_newName[0])
            self.sensor_strategy.setPropValue("regionCompleted", self.proj.rfi.regions[curRegionIdx])

        if self.proj.compile_options['neighbour_robot'] and self.proj.compile_options["multi_robot_mode"] == "patching":
            # ************ patching ****************** #
            env_assumption_hold = self.checkEnvTransViolationWithNextPossibleStates(self.LTLViolationCheckPossibleStates)
            # **************************************** #

        elif self.proj.compile_options['neighbour_robot'] and self.proj.compile_options["multi_robot_mode"] == "d-patching":
            # %%%%%%%%%%%%% d-patching %%%%%%%%%%%%%%% #
            if self.runCentralizedStrategy:
                for prop_name, value in self.dPatchingExecutor.convertFromRegionBitsToRegionNameInDict('env', self.sensor_strategy.getInputs(expand_domains = True)).iteritems():
                    self.dPatchingExecutor.sensor_state.setPropValue(self.dPatchingExecutor.propMappingOldToNew[self.dPatchingExecutor.robotName][prop_name], value)

                #env_assumption_hold = self.globalEnvTransCheck.checkViolation(self.dPatchingExecutor.strategy.current_state, self.dPatchingExecutor.sensor_state)
                deepcopy_sensor_state = copy.deepcopy(self.dPatchingExecutor.sensor_state)
                deepcopy_current_state = copy.deepcopy(self.dPatchingExecutor.strategy.current_state)
                env_assumption_hold = self.checkEnvTransViolationWithNextPossibleStates(checker=self.globalEnvTransCheck,\
                    current_state=deepcopy_current_state, sensor_state=deepcopy_sensor_state)

                if not env_assumption_hold:
                    ltlmop_logger.debug("======== envTrans violations detected ============")
                    ltlmop_logger.debug("sensor_state:" + str([x for x, value in deepcopy_sensor_state.getInputs().iteritems() if value]))
                    ltlmop_logger.debug("env_assumption_hold:" + str(env_assumption_hold))

                    # now checks if it's only about one coordinating robot
                    for x in self.globalEnvTransCheck.violated_specStr:
                        if LTLParser.LTLcheck.filterSpecList([x], self.dPatchingExecutor.robotInRange + [self.dPatchingExecutor.robotName]):
                            break
                    else:
                        env_assumption_hold = True
                        ltlmop_logger.debug("no violations as it's only about one robot (later should change to only topology)")
            else:
                env_assumption_hold = self.checkEnvTransViolationWithNextPossibleStates(self.LTLViolationCheckPossibleStates)
            #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% #
        else:
            # Check for environment violation - change the env_assumption_hold to int again
            env_assumption_hold = self.LTLViolationCheckPossibleStates.checkViolation(copy.deepcopy(self.strategy.current_state), copy.deepcopy(self.sensor_strategy))
        ltlmop_logger.warning("checked all possible states")

        return env_assumption_hold

    def run_check_envTrans_violations(self):
        """
        This function puts checkEnvTransViolationWithNextPossibleStates into a while loop for using with thread
        """
        sensor_state = None
        otherEnvPropDict = None
        current_state = None

        old_sensor_state = None
        old_otherEnvPropDict = None
        old_current_state = None
        try:
            while self.alive.isSet():
                if self.runStrategy.isSet() and self.runRuntimeMonitoring.isSet():
                    # only run check violation if sensor or possible next states has changed
                    runCheck = False
                    #ltlmop_logger.warning("how often do we come.....possible states")
                    sensor_state = copy.deepcopy(self.hsub.getSensorValue(self.proj.enabled_sensors))
                    if self.proj.compile_options['neighbour_robot'] and self.proj.compile_options["multi_robot_mode"] == "patching":
                        otherEnvPropDict = self.robClient.requestNextPossibleEnvStatesFromOtherRobot()
                    elif self.proj.compile_options['neighbour_robot'] and self.proj.compile_options["multi_robot_mode"] == "d-patching":
                        otherEnvPropDict = self.dPatchingExecutor.getNextPossibleEnvStatesFromOtherRobots()

                    # figure out the form of current strategy
                    if self.runCentralizedStrategy:
                        current_state = copy.deepcopy(self.dPatchingExecutor.strategy.current_state)
                    else:
                        current_state = copy.deepcopy(self.strategy.current_state)

                    if (old_current_state is None or old_current_state.getAll(expand_domains=True) != current_state.getAll(expand_domains=True)) or\
                        old_sensor_state != sensor_state or old_otherEnvPropDict != otherEnvPropDict:
                        runCheck = True

                    if runCheck and self.runStrategy.isSet() and self.runRuntimeMonitoring.isSet():
                        self.possible_states_env_assumption_hold = self.check_envTrans_violations()
                        # stop check violations until this one is solved.
                        if not self.possible_states_env_assumption_hold:
                            self.runRuntimeMonitoring.clear()

                        if self.runCentralizedStrategy:
                            self.possible_states_violated_spec_list = copy.deepcopy(self.globalEnvTransCheck.violated_specStr)
                            self.possible_states_violated_spec_list_with_no_specText_match = copy.deepcopy(self.globalEnvTransCheck.violated_specStr_with_no_specText_match)
                            self.possible_states_violated_spec_line_no = copy.deepcopy(self.globalEnvTransCheck.violated_spec_line_no)
                        else:
                            self.possible_states_violated_spec_list = copy.deepcopy(self.LTLViolationCheckPossibleStates.violated_specStr)
                            self.possible_states_violated_spec_list_with_no_specText_match = copy.deepcopy(self.LTLViolationCheckPossibleStates.violated_specStr_with_no_specText_match)
                            self.possible_states_violated_spec_line_no = copy.deepcopy(self.LTLViolationCheckPossibleStates.violated_spec_line_no)

                        ltlmop_logger.debug("**self.possible_states_violated_spec_list:" + str(self.possible_states_violated_spec_list))
                        ltlmop_logger.debug("self.possible_states_env_assumption_hold:" + str(self.possible_states_env_assumption_hold))

                    old_sensor_state = copy.deepcopy(sensor_state)
                    old_otherEnvPropDict = copy.deepcopy(otherEnvPropDict)
                    old_current_state = copy.deepcopy(current_state)

        except:
            import traceback
            ltlmop_logger.error(traceback.format_exc())

    def resumeRuntimeMonitoring(self):
        """
        This function clears all violations related variables and set the runtime thread again.
        """
        # reset previous variables
        self.old_violated_specStr = []
        self.old_violated_specStr_with_no_specText_match = []
        self.old_violated_spec_line_no = []

        self.old_possible_states_violated_specStr_with_no_specText_match = []
        self.old_possible_states_violated_spec_line_no = []

        # reset variables
        self.env_assumption_hold = True
        self.violated_spec_list = []
        self.violated_spec_list_with_no_specText_match = []
        self.violated_spec_line_no = []

        self.possible_states_env_assumption_hold = True # keeping track if env trans holds with/without possible next states.
        self.possible_states_violated_spec_list = [] # keep track of specs violated
        self.possible_states_violated_spec_list_with_no_specText_match = [] # keep track of specs violated but with no specText match
        self.possible_states_violated_spec_line_no = [] # keep track of specText lines violated.

        # also reset those in the object
        if self.runCentralizedStrategy:
            if self.globalEnvTransCheck:
                self.globalEnvTransCheck.clearViolations()
        else:
            if self.LTLViolationCheckPossibleStates:
                self.LTLViolationCheckPossibleStates.clearViolations()
            if self.LTLViolationCheck:
                self.LTLViolationCheck.clearViolations()

        #resume the thread
        self.runRuntimeMonitoring.set()
