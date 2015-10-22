import logging
import time
import copy
import LTLParser.LTLcheck

class ExecutorModesExtensions(object):
    """ Extensions to Executor to allow for specification rewriting and resynthesis.
        This class is not meant to be instantiated. """

    def __init__(self):
        super(ExecutorModesExtensions, self).__init__()
        logging.info("Initializing modes extensions...")

        self.env_assumption_hold = True # keeping track if env trans holds with/without possible next states.
        self.violated_spec_list = [] # keep track of specs violated
        self.violated_spec_list_with_no_specText_match = [] # keep track of specs violated but with no specText match
        self.violated_spec_line_no = [] # keep track of specText lines violated.
        #self.violation_resolved = True # make sure violation holds until they are resolved.

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
            curRegionIdx = self._getCurrentRegionFromPose()
            if curRegionIdx is None:
                curRegionIdx = self.proj.rfi.indexOfRegionWithName(decomposed_region_names[0])
            self.sensor_strategy.setPropValue("regionCompleted", self.proj.rfi.regions[curRegionIdx])

        if self.proj.compile_options['neighbour_robot'] and self.proj.compile_options["multi_robot_mode"] == "patching":
            # ************ patching ****************** #
            env_assumption_hold = self.LTLViolationCheck.checkViolation(self.strategy.current_state, self.sensor_strategy)
            if env_assumption_hold:
                env_assumption_hold = self.checkEnvTransViolationWithNextPossibleStates()
            # **************************************** #

        elif self.proj.compile_options['neighbour_robot'] and self.proj.compile_options["multi_robot_mode"] == "d-patching":
            # %%%%%%%%%%%%% d-patching %%%%%%%%%%%%%%% #
            if self.runCentralizedStrategy:
                for prop_name, value in self.dPatchingExecutor.convertFromRegionBitsToRegionNameInDict('env', self.sensor_strategy.getInputs(expand_domains = True)).iteritems():
                    self.dPatchingExecutor.sensor_state.setPropValue(self.dPatchingExecutor.propMappingOldToNew[self.dPatchingExecutor.robotName][prop_name], value)

                #env_assumption_hold = self.globalEnvTransCheck.checkViolation(self.dPatchingExecutor.strategy.current_state, self.dPatchingExecutor.sensor_state)
                env_assumption_hold = self.checkEnvTransViolationWithNextPossibleStates(checker=self.globalEnvTransCheck,\
                    current_state=self.dPatchingExecutor.strategy.current_state, sensor_state=self.dPatchingExecutor.sensor_state)

                if not env_assumption_hold:
                    logging.debug("sensor_state:" + str([x for x, value in self.dPatchingExecutor.sensor_state.getInputs().iteritems() if value]))
                    logging.debug("env_assumption_hold:" + str(env_assumption_hold))
                    logging.debug("======== envTrans violations detected ============")

                    # now checks if it's only about one coordinating robot
                    for x in self.globalEnvTransCheck.violated_specStr:
                        if LTLParser.LTLcheck.filterSpecList([x], self.dPatchingExecutor.robotInRange + [self.dPatchingExecutor.robotName]):
                            break
                    else:
                        env_assumption_hold = True
                        logging.debug("no violations as it's only about one robot (later should change to only topology)")
            else:
                env_assumption_hold = self.LTLViolationCheck.checkViolation(self.strategy.current_state, self.sensor_strategy)
                if env_assumption_hold:
                    env_assumption_hold = self.checkEnvTransViolationWithNextPossibleStates()
            #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% #
        else:
            # Check for environment violation - change the env_assumption_hold to int again
            env_assumption_hold = self.LTLViolationCheck.checkViolation(self.strategy.current_state, self.sensor_strategy)

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
                    sensor_state = self.hsub.getSensorValue(self.proj.enabled_sensors)
                    if self.proj.compile_options['neighbour_robot'] and self.proj.compile_options["multi_robot_mode"] == "patching":
                        otherEnvPropDict = self.robClient.requestNextPossibleEnvStatesFromOtherRobot()
                    elif self.proj.compile_options['neighbour_robot'] and self.proj.compile_options["multi_robot_mode"] == "d-patching":
                        otherEnvPropDict = self.dPatchingExecutor.getNextPossibleEnvStatesFromOtherRobots()

                    # figure out the form of current strategy
                    if self.runCentralizedStrategy:
                        current_state = self.dPatchingExecutor.strategy.current_state
                    else:
                        current_state = self.strategy.current_state

                    if old_current_state != current_state or old_sensor_state != sensor_state or old_otherEnvPropDict != otherEnvPropDict:
                        runCheck = True

                    if runCheck and self.runStrategy.isSet() and self.runRuntimeMonitoring.isSet():
                        self.env_assumption_hold = self.check_envTrans_violations()
                        # stop check violations until this one is solved.
                        if not self.env_assumption_hold:
                            self.runRuntimeMonitoring.clear()

                        if self.runCentralizedStrategy:
                            self.violated_spec_list = copy.deepcopy(self.globalEnvTransCheck.violated_specStr)
                            self.violated_spec_list_with_no_specText_match = copy.deepcopy(self.globalEnvTransCheck.violated_specStr_with_no_specText_match)
                            self.violated_spec_line_no = copy.deepcopy(self.globalEnvTransCheck.violated_spec_line_no)
                        else:
                            self.violated_spec_list = copy.deepcopy(self.LTLViolationCheck.violated_specStr)
                            self.violated_spec_list_with_no_specText_match = copy.deepcopy(self.LTLViolationCheck.violated_specStr_with_no_specText_match)
                            self.violated_spec_line_no = copy.deepcopy(self.LTLViolationCheck.violated_spec_line_no)

                        logging.debug("**self.violated_spec_list:" + str(self.violated_spec_list))
                        logging.debug("self.env_assumption_hold:" + str(self.env_assumption_hold))

                    old_sensor_state = copy.deepcopy(sensor_state)
                    old_otherEnvPropDict = copy.deepcopy(otherEnvPropDict)
                    old_current_state = current_state

        except:
            import traceback
            logging.error(traceback.format_exc())

    def resumeRuntimeMonitoring(self):
        """
        This function clears all violations related variables and set the runtime thread again.
        """
        # reset variables
        self.env_assumption_hold = True
        self.violated_spec_list = []
        self.violated_spec_list_with_no_specText_match = []
        self.violated_spec_line_no = []

        #resume the thread
        self.runRuntimeMonitoring.set()
