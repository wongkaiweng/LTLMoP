import fsa
import sys
import random
import project
import copy

# logger for ltlmop
import logging
ltlmop_logger = logging.getLogger('ltlmop_logger')

class ExecutorStrategyExtensions(object):
    """ Extensions to Executor to allow for the strategy structure.
        This class provides functions to update the outputs and to check for new states in every iteration.
    """

    def __init__(self):
        super(ExecutorStrategyExtensions, self).__init__()

        self.last_next_states= []
        self.last_sensor_state = {}
        self.next_state      = None
        self.current_region  = None
        self.next_region     = None

    def updateOutputs(self, state=None):
        """
        Update the values of current outputs in our execution environment to reflect the output
        proposition values associated with the given state
        """

        if state is None:
            state = self.current_state

        for key, output_val in state.getOutputs().iteritems():
            # Skip any region
            if 'region' == key: continue

            if key not in self.current_outputs.keys() or output_val != self.current_outputs[key]:

                # The state of this output proposition has changed!
                self.postEvent("INFO", "Output proposition \"%s\" is now %s!" % (key, str(output_val)))

                # Run any actuator handlers if appropriate
                if key in self.proj.enabled_actuators:
                    self.hsub.setActuatorValue({key:output_val})

                self.current_outputs[key] = output_val

    def runStrategyIteration(self):
        """
        Run, run, run the automaton!  (For one evaluation step)
        """
        if self.proj.rfi:
            # find current region
            self.current_region = self.strategy.current_state.getPropValue('region')
        else:
            self.current_region = None

        # Take a snapshot of our current sensor readings
        sensor_state = self.hsub.getSensorValue(self.proj.enabled_sensors)

        # Let's try to transition
        # TODO: set current state so that we don't need to call from_state
        next_states = self.strategy.findTransitionableStates(sensor_state, from_state= self.strategy.current_state)

        # Make sure we have somewhere to go
        if len(next_states) == 0:
            # Well darn!
            ltlmop_logger.error("Could not find a suitable state to transition to!")
            return

        # See if we're beginning a new transition
        if next_states != self.last_next_states:
            # NOTE: The last_next_states comparison is also to make sure we don't
            # choose a different random next-state each time, in the case of multiple choices

            self.last_next_states = next_states

            # Only allow self-transitions if that is the only option!
            if len(next_states) > 1 and self.strategy.current_state in next_states:
                next_states.remove(self.strategy.current_state)

            self.next_state = random.choice(next_states)
            if self.proj.rfi:
                self.next_region = self.next_state.getPropValue('region')
            else:
                self.next_region = None
            
            if self.next_state.goal_id != self.strategy.current_state.goal_id:
                self.postEvent("INFO", "Currently pursuing goal #{}".format(self.next_state.goal_id))

            # See what we, as the system, need to do to get to this new state
            self.transition_contains_motion = self.next_region is not None and (self.next_region != self.current_region)

            if self.transition_contains_motion:
                # We're going to a new region
                self.postEvent("INFO", "Heading to region %s..." % self.next_region.name)

            self.arrived = False

        if not self.arrived:
            if self.proj.rfi:
                # Move one step towards the next region (or stay in the same region)
                self.arrived = self.hsub.gotoRegion(self.current_region, self.next_region)
            else:
                self.arrived = True

        # Check for completion of motion
        if self.arrived and self.next_state != self.strategy.current_state:
            # TODO: Check to see whether actually inside next region that we expected
            if self.transition_contains_motion:
                self.postEvent("INFO", "Crossed border from %s to %s!" % (self.current_region.name, self.next_region.name))

            if not self.proj.compile_options['fastslow']:
                # Run actuators after motion
                self.updateOutputs(self.next_state)
            
            # ------------------------------- #
            # --- two_robot_negotiation ----- #
            # ------------------------------- #
            if self.proj.compile_options['neighbour_robot']:
                self.robClient.updateRobotRegion(self.next_region)
            # ------------------------------- #
            
            self.strategy.current_state = self.next_state
            self.last_next_states = []  # reset

            self.postEvent("INFO", "Now in state %s (z = %s)" % (self.strategy.current_state.state_id, self.strategy.current_state.goal_id))

    def runStrategyIterationInstanteousAction(self):
        """
        Run, run, run the strategy with instantanous action!  (For one evaluation step)
        """
        # Take a snapshot of our current sensor readings
        sensor_state = self.hsub.getSensorValue(self.proj.enabled_sensors)

        # find current region based on region sensors and remove those sensors from sensor_state
        # finally add the "regionCompleted" sensor with region object
        if self.proj.rfi:
            sensor_region = dict((k,v) for k, v in  sensor_state.iteritems() if k.endswith('_rc') and not k.startswith(tuple(self.proj.otherRobot)))
            for key, value in sensor_region.iteritems():
                del sensor_state[key]
            sensor_region_names = [k for k, v in  sensor_region.iteritems() if v]

            # check we are inside any region, if not we will use the previous region detected. (timing issue)
            if sensor_region_names:
                sensor_region_name = sensor_region_names[0].replace('_rc','')
                if self.proj.compile_options["decompose"]:
                    decomposed_region_names = self.proj.regionMapping[sensor_region_name]
                else: # use original name, not p#
                    decomposed_region_names = [sensor_region_name]

                self.prev_decomposed_region_names = decomposed_region_names
            else:
                ltlmop_logger.info('not inside any region!')
                decomposed_region_names  = self.prev_decomposed_region_names

            sensor_state['regionCompleted'] = self.proj.rfi.regions[self.proj.rfi.indexOfRegionWithName(decomposed_region_names[0])]  #should only be one in our case. not taking care of convexify now
            self.current_region = sensor_state['regionCompleted']
        else:
            self.current_region = None

        # Let's try to transition
        # TODO: set current state so that we don't need to call from_state
        next_states = self.strategy.findTransitionableStates(sensor_state, from_state= self.strategy.current_state)

        # check assumptions here only for negotitation
        if self.proj.compile_options['neighbour_robot'] and self.proj.compile_options["multi_robot_mode"] == "negotiation":
            self.env_assumption_hold = self.simple_check_envTrans_violation()
            #!!! this is only for negotiation. also update snesor info and regions
            if not self.env_assumption_hold and sensor_state != self.last_sensor_state:
                self.last_sensor_state = sensor_state
                if self.proj.compile_options['include_heading']:
                    self.robClient.updateCompletedRobotRegion(sensor_state('regionCompleted'))
                else:
                    self.robClient.updateRobotRegion(sensor_state['regionCompleted'])

                # also update sensors
                tempDict = {k: v for k, v in sensor_state.iteritems()\
                    if k != 'regionCompleted' and not k.startswith(tuple(self.proj.otherRobot)) and not k.endswith('_rc')}
                tempDict.update(self.strategy.current_state.getOutputs(expand_domains=True))
                self.robClient.updateRobotSensors(tempDict)

                if len(next_states) == 0:
                    # Well darn!
                    ltlmop_logger.error("Could not find a suitable state to transition to!")
                return

        # Make sure we have somewhere to go
        if len(next_states) == 0:
            # Well darn!
            ltlmop_logger.error("Could not find a suitable state to transition to!")

            if self.proj.compile_options['recovery']:
                # set violationCount to be violationThres such that envChar is triggered
                self.envViolationCount = self.envViolationThres+1

            # %%%%%%%%%%%%  d-patching %%%%%%%%%%% #
            # update current region even though no next state is found.
            if self.proj.compile_options['neighbour_robot']:
                if self.proj.compile_options["multi_robot_mode"] == "d-patching":
                    if self.proj.compile_options['include_heading']:
                        self.dPatchingExecutor.updateCompletedRobotRegionWithAllClients(sensor_state('regionCompleted'))
                    else:
                        self.dPatchingExecutor.updateRobotRegionWithAllClients(sensor_state['regionCompleted'])

                    # also update sensors
                    enabled_sensors = [x for x in self.proj.enabled_sensors if not (x.endswith('_rc') or x.startswith(tuple(self.dPatchingExecutor.robotInRange)))]
                    ltlmop_logger.warning('{x:sensor_state[x] for x in enabled_sensors}:' + str({x:sensor_state[x] for x in enabled_sensors}))
                    self.dPatchingExecutor.updateRobotSensorsWithAllClients({x:sensor_state[x] for x in enabled_sensors})

                elif self.proj.compile_options["multi_robot_mode"] == "negotiation":

                    if self.proj.compile_options['include_heading']:
                        self.robClient.updateCompletedRobotRegion(sensor_state('regionCompleted'))
                    else:
                        self.robClient.updateRobotRegion(sensor_state['regionCompleted'])

                    # also update sensors
                    tempDict = {k: v for k, v in sensor_state.iteritems()\
                        if k != 'regionCompleted' and not k.startswith(tuple(self.proj.otherRobot)) and not k.endswith('_rc')}
                    tempDict.update(self.strategy.current_state.getOutputs(expand_domains=True))
                    self.robClient.updateRobotSensors(tempDict)

            return

        # See if we're beginning a new transition
        if next_states != self.last_next_states:
            # NOTE: The last_next_states comparison is also to make sure we don't
            # choose a different random next-state each time, in the case of multiple choices
            self.last_next_states = next_states

            # Only allow self-transitions if that is the only option!
            if len(next_states) > 1 and self.strategy.current_state in next_states:
                next_states.remove(self.strategy.current_state)

            self.next_state = random.choice(next_states)
            # find next region
            if self.proj.rfi:
                self.next_region = self.next_state.getPropValue('region')
            self.postEvent("INFO", "Currently pursuing goal #{}".format(self.next_state.goal_id))
            ltlmop_logger.info("Currently at state %s." % self.next_state.state_id)

            # ------------------------------- #
            # --- two_robot_negotiation ----- #
            # ------------------------------- #
            # without include_heading: updateRobotRegion sends the current location of the robot
            # with include_heading: updateRobotRegion sends the heading region of the robot
            if self.proj.compile_options['neighbour_robot'] and self.proj.compile_options['include_heading']:
                if self.proj.compile_options["multi_robot_mode"] == "d-patching":
                    # update region info for all connected robots.
                    self.dPatchingExecutor.updateRobotRegionWithAllClients(self.next_region)
                else:
                    self.robClient.updateRobotRegion(self.next_region)

            if self.proj.compile_options['neighbour_robot'] and self.proj.compile_options["multi_robot_mode"] == "negotiation":
                tempDict = {k: v for k, v in sensor_state.iteritems()\
                    if k != 'regionCompleted' and not k.startswith(tuple(self.proj.otherRobot)) and not k.endswith('_rc')}
                tempDict.update(self.next_state.getOutputs(expand_domains=True))
                self.robClient.updateRobotSensors(tempDict)
            # ------------------------------- #

            # See what we, as the system, need to do to get to this new state
            self.transition_contains_motion = self.next_region is not None and (self.next_state.getPropValue('regionCompleted') != self.strategy.current_state.getPropValue('regionCompleted'))

            # Run actuators before motion
            self.updateOutputs(self.next_state)

            # save current sensor state for next iteration
            self.last_sensor_state = sensor_state

            # Check for completion of motion
            #if self.next_state.getAll(expand_domains=True) != self.strategy.current_state.getAll(expand_domains=True):
            # *********** patching *********** #
            if self.proj.compile_options['neighbour_robot'] and \
               (self.proj.compile_options["multi_robot_mode"] == "patching" or self.proj.compile_options["multi_robot_mode"] == "d-patching"):

                # first find all next possible states
                if self.proj.compile_options['symbolic'] or self.proj.compile_options['interactive']:
                    possible_next_states = self.strategy.findTransitionableNextStates(from_state=self.next_state)
                    """
                    statesToConsider = self.strategy.findTransitionableNextStates(from_state=self.next_state)
                    #ltlmop_logger.debug('statesToConsider:' + str(statesToConsider))
                    possible_next_states = []
                    for state in statesToConsider:
                        ltlmop_logger.warning([k for k, v in state.getAll(expand_domains=True).iteritems() if v])
                    """
                else:
                    possible_next_states = self.strategy.findTransitionableStates({}, from_state=self.next_state)

                # update current next states sent tothe other robot
                if self.proj.compile_options["multi_robot_mode"] == "patching":
                    self.robClient.sendNextPossibleEnvStatesToOtherRobot(possible_next_states)
                else: #d-patching
                    self.dPatchingExecutor.sendNextPossibleEnvStatesToOtherRobotToAllClients(possible_next_states)

                # also update action completions
                self.dPatchingExecutor.updateRobotActionStatusWithAllClients({k.replace('_ac',''):v for k, v in self.next_state.getInputs(expand_domains=True).iteritems() if k.endswith('_ac')})
                # update our own dPatching
                self.dPatchingExecutor.actionStatus[self.dPatchingExecutor.robotName] = {k.replace('_ac',''):v for k, v in self.next_state.getInputs(expand_domains=True).iteritems() if k.endswith('_ac')}
            # ******************************** #

            """
            if self.proj.compile_options['neighbour_robot'] and self.proj.compile_options["multi_robot_mode"] == "negotiation":
                if self.proj.compile_options['interactive']:
                   possible_next_states = self.strategy.findTransitionableNextStates(from_state=self.next_state)

                   statesToConsider = self.strategy.findTransitionableNextStates(from_state=self.next_state)
                   #ltlmop_logger.debug('statesToConsider:' + str(statesToConsider))
                   possible_next_states = []
                   for state in statesToConsider:
                       envTrans_hold = self.envTransCheck.checkViolation(self.next_state, state)
                       sysTrans_hold = self.sysTransCheck.checkViolation(self.next_state, state)
                       if envTrans_hold and sysTrans_hold:
                           ltlmop_logger.warning([k for k, v in state.getAll(expand_domains=True).iteritems() if v])
                else:
                    try:
                        possible_next_states = self.strategy.findTransitionableStates({}, from_state=self.next_state)
                        for state in possible_next_states:
                            ltlmop_logger.warning([k for k, v in state.getAll(expand_domains=True).iteritems() if v])
                    except:
                        pass
            """

            if self.transition_contains_motion:
                self.postEvent("INFO", "Crossed border from %s to %s!" % (self.strategy.current_state.getPropValue('regionCompleted').name, self.next_state.getPropValue('regionCompleted').name))

                # ------------------------------- #
                # --- two_robot_negotiation ----- #
                # ------------------------------- #
                if self.proj.compile_options['neighbour_robot']:
                    if self.proj.compile_options["multi_robot_mode"] == "d-patching":
                        if self.proj.compile_options['include_heading']:
                            self.dPatchingExecutor.updateCompletedRobotRegionWithAllClients(self.next_state.getPropValue('regionCompleted'))
                        else:
                            self.dPatchingExecutor.updateRobotRegionWithAllClients(self.next_state.getPropValue('regionCompleted'))
                    else:
                        if self.proj.compile_options['include_heading']:
                            self.robClient.updateCompletedRobotRegion(self.next_state.getPropValue('regionCompleted'))
                        else:
                            self.robClient.updateRobotRegion(self.next_state.getPropValue('regionCompleted'))
                # ------------------------------- #

            if self.transition_contains_motion or self.next_region is not None and (self.next_state.getPropValue('region') != self.strategy.current_state.getPropValue('region')):
                self.postEvent("INFO", "Heading to region %s..." % self.next_state.getPropValue('region').name)

            self.strategy.current_state = self.next_state
            #self.last_next_states = []  # reset

            if not self.proj.compile_options['interactive']:
                self.postEvent("INFO", "Now in state %s (z = %s)" % (self.strategy.current_state.state_id, self.strategy.current_state.goal_id))
            else:
                ltlmop_logger.info(str([prop for prop, value in self.strategy.current_state.getAll(expand_domains=True).iteritems() if value]))
                ltlmop_logger.info('---------------------------------------------------------')

        if self.strategy.current_state.getAll(expand_domains=True) == self.next_state.getAll(expand_domains=True):
            ##########################################
            #### CHECK IF WE ARE ASKED TO PAUSE ######
            ##########################################
            if self.proj.rfi:
                if self.proj.compile_options["multi_robot_mode"] == "d-patching" and self.dPatchingExecutor.getPauseForControllerSynthesis():
                    # we are asked to pause
                    self.hsub.setVelocity(0, 0)
                else:
                    # Move one step towards the next region (or stay in the same region)
                    self.hsub.gotoRegion(self.current_region, self.next_region)


    def runStrategyIterationInstanteousActionCentralized(self):
        """
        Run, run, run the strategy with instantanous action!  (For one evaluation step)
        For patching mode
        """
        # Take a snapshot of our current sensor readings
        sensor_state = self.hsub.getSensorValue(self.proj.enabled_sensors)

        # find current region based on region sensors and remove those sensors from sensor_state
        # finally add the "regionCompleted" sensor with region object
        sensor_region = dict((k,v) for k, v in  sensor_state.iteritems() if k.endswith('_rc') and not k.startswith(tuple(self.proj.otherRobot)))
        for key, value in sensor_region.iteritems():
            del sensor_state[key]
        sensor_region_names = [k for k, v in  sensor_region.iteritems() if v]

        # check we are inside any region, if not we will use the previous region detected. (timing issue)
        if sensor_region_names:
            sensor_region_name = sensor_region_names[0].replace('_rc','')
            decomposed_region_names = self.proj.regionMapping[sensor_region_name]
            self.prev_decomposed_region_names = decomposed_region_names
        else:
            ltlmop_logger.info('not inside any region!')
            decomposed_region_names  = self.prev_decomposed_region_names

        sensor_state['regionCompleted'] = self.proj.rfi.regions[self.proj.rfi.indexOfRegionWithName(decomposed_region_names[0])]  #should only be one in our case. not taking care of convexify now
        self.current_region = sensor_state['regionCompleted']

        # temporarily save the centralized state to our strategy
        self.centralized_strategy_state = self.strategy.states.addNewState()
        self.centralized_strategy_state.setPropValues(sensor_state)

        # also set current goal to be the same as before
        self.centralized_strategy_state.goal_id = self.strategy.current_state.goal_id

        # update the environment propositions of centralized strategy
        # AND return latest system propositions of the robot
        if self.proj.compile_options["multi_robot_mode"] == "d-patching":
            sysOutputs = self.dPatchingExecutor.getOutputs(self.centralized_strategy_state.getInputs(expand_domains=True))
        elif self.proj.compile_options["multi_robot_mode"] == "patching":
            sysOutputs = self.robClient.getOutputs(self.centralized_strategy_state.getInputs(expand_domains=True))

        # Make sure we have somewhere to go
        if len(sysOutputs) == 0:
            # Well darn!
            ltlmop_logger.error("Could not find a suitable state to transition to in centralized strategy!")

            # %%%%%%%%%%%%  d-patching %%%%%%%%%%% #
            # update current region even though no next state is found.
            if self.proj.compile_options['neighbour_robot']:
                if self.proj.compile_options["multi_robot_mode"] == "d-patching":
                    if self.proj.compile_options['include_heading']:
                        self.dPatchingExecutor.updateCompletedRobotRegionWithAllClients(sensor_state('regionCompleted'))
                    else:
                        self.dPatchingExecutor.updateRobotRegionWithAllClients(sensor_state['regionCompleted'])

                    # also update sensors
                    enabled_sensors = [x for x in self.proj.enabled_sensors if not (x.endswith('_rc') or x.startswith(tuple(self.dPatchingExecutor.robotInRange)))]
                    ltlmop_logger.warning('{x:sensor_state[x] for x in enabled_sensors}:' + str({x:sensor_state[x] for x in enabled_sensors}))
                    self.dPatchingExecutor.updateRobotSensorsWithAllClients({x:sensor_state[x] for x in enabled_sensors})

                elif self.proj.compile_options["multi_robot_mode"] == "negotiation":
                    if self.proj.compile_options['include_heading']:
                        self.robClient.updateCompletedRobotRegion(sensor_state('regionCompleted'))
                    else:
                        self.robClient.updateRobotRegion(sensor_state['regionCompleted'])

            # check if goals are reached
            if not self.dPatchingExecutor.checkSysGoalsThread or not self.dPatchingExecutor.checkSysGoalsThread.isAlive():
                self.dPatchingExecutor.runSingleTime_checkIfGoalsAreSatisfied()
            # %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% #
            return

        # temporarily save the centralized state to our strategy
        self.centralized_strategy_state.setPropValues(sysOutputs)

        # See if we're beginning a new transition
        #if self.strategy.current_state.getOutputs() != sysOutputs:
        if self.strategy.current_state.getAll(expand_domains=True) != self.centralized_strategy_state.getAll(expand_domains=True):

            # find next region
            self.next_region = sysOutputs['region']
            # ------------------------------- #
            # --- two_robot_negotiation ----- #
            # ------------------------------- #
            # without include_heading: updateRobotRegion sends the current location of the robot
            # with include_heading: updateRobotRegion sends the heading region of the robot
            if self.proj.compile_options['neighbour_robot']:
                if self.proj.compile_options["multi_robot_mode"] == "d-patching":
                    if self.proj.compile_options['include_heading']:
                        # update region info for all connected robots.
                        self.dPatchingExecutor.updateRobotRegionWithAllClients(self.next_region)
                        self.dPatchingExecutor.updateCompletedRobotRegionWithAllClients(sensor_state['regionCompleted'])
                    else:
                        self.dPatchingExecutor.updateRobotRegionWithAllClients(sensor_state['regionCompleted'])

                    # also update sensors
                    enabled_sensors = [x for x in self.proj.enabled_sensors if not (x.endswith('_rc') or x.startswith(tuple(self.dPatchingExecutor.robotInRange)))]
                    ltlmop_logger.warning('{x:sensor_state[x] for x in enabled_sensors}:' + str({x:sensor_state[x] for x in enabled_sensors}))
                    self.dPatchingExecutor.updateRobotSensorsWithAllClients({x:sensor_state[x] for x in enabled_sensors})
                else:
                    if self.proj.compile_options['include_heading']:
                        self.robClient.updateRobotRegion(self.next_region)
                        self.robClient.updateCompletedRobotRegion(sensor_state['regionCompleted'])
                    else:
                        self.robClient.updateRobotRegion(sensor_state['regionCompleted'])
            # ------------------------------- #

            # *********** patching *********** #
            if self.proj.compile_options['neighbour_robot'] and self.proj.compile_options["multi_robot_mode"] == "d-patching":

                # first find all next possible states
                if self.dPatchingExecutor.compiler.proj.compile_options['symbolic'] or self.dPatchingExecutor.compiler.proj.compile_options['interactive']:
                    possible_next_states = self.dPatchingExecutor.strategy.findTransitionableNextStates(from_state=self.dPatchingExecutor.strategy.current_state)
                    """
                    statesToConsider = self.strategy.findTransitionableNextStates(from_state=self.next_state)
                    #ltlmop_logger.debug('statesToConsider:' + str(statesToConsider))
                    possible_next_states = []
                    for state in statesToConsider:
                        # also checks if states satisfy phi_e^t and phi_s^t
                        envTrans_hold = self.envTransCheck.checkViolation(self.next_state, state)
                        sysTrans_hold = self.sysTransCheck.checkViolation(self.next_state, state)
                        if envTrans_hold and sysTrans_hold:
                            possible_next_states.append(state)
                    """
                else:
                    possible_next_states = self.dPatchingExecutor.strategy.findTransitionableStates({}, from_state=self.dPatchingExecutor.strategy.current_state)

                # update current next states sent tothe other robot
                self.dPatchingExecutor.sendNextPossibleEnvStatesPreparedToOtherRobotToAllClients(possible_next_states)

                # also update action completions
                self.dPatchingExecutor.updateRobotActionStatusWithAllClients({k.replace('_ac',''):v for k, v in self.centralized_strategy_state.getInputs(expand_domains=True).iteritems() if k.endswith('_ac')})
                # update our own dPatching
                self.dPatchingExecutor.actionStatus[self.dPatchingExecutor.robotName] = {k.replace('_ac',''):v for k, v in self.centralized_strategy_state.getInputs(expand_domains=True).iteritems() if k.endswith('_ac')}
            # ******************************** #

            # See what we, as the system, need to do to get to this new state
            self.transition_contains_motion = self.next_region is not None and (self.strategy.current_state.getPropValue('regionCompleted')!= self.centralized_strategy_state.getPropValue('regionCompleted'))

            # Run actuators before motion
            self.updateOutputs(self.centralized_strategy_state)

            # Check for completion of motion
            if self.transition_contains_motion:
                self.postEvent("INFO", "Crossed border from %s to %s!" % (self.strategy.current_state.getPropValue('regionCompleted').name, self.centralized_strategy_state.getPropValue('regionCompleted').name))

            if self.transition_contains_motion or self.next_region is not None and (self.strategy.current_state.getPropValue('region')!= self.centralized_strategy_state.getPropValue('region')):
                self.postEvent("INFO", "Heading to region %s..." % self.next_region.name)

            self.strategy.current_state = copy.deepcopy(self.centralized_strategy_state)

        ##########################################
        #### CHECK IF WE ARE ASKED TO PAUSE ######
        ##########################################
        if self.proj.compile_options["multi_robot_mode"] == "d-patching" and self.dPatchingExecutor.getPauseForControllerSynthesis():
            # we are asked to pause
            self.hsub.setVelocity(0, 0)
        elif self.strategy.current_state.getPropValue('regionCompleted') == self.strategy.current_state.getPropValue('region'):
            self.hsub.setVelocity(0, 0)
        else:
            # Move one step towards the next region (or stay in the same region)
            self.hsub.gotoRegion(self.strategy.current_state.getPropValue('regionCompleted'), self.strategy.current_state.getPropValue('region'))

    def HSubGetSensorValue(self,sensorList):
        """
        This function takes in a list of sensorName and returns the dictionary of the propositions with values.
        This will be replaced by a function in handlerSubsystem.py in the future.
        Input:
        sensorList (list): list of the sensor propositions

        Output:
        sensor_state (dict): dictionary containing the sensor name with the corresponding sensor value.
        """
        self.h_instance = self.proj.h_instance
        sensor_state = {}
        for sensor in self.proj.enabled_sensors:
            sensor_state[sensor]  = eval(self.proj.sensor_handler[sensor], {'self':self,'initial':False})
        return sensor_state

    def HSubSetActuatorValue(self, actuatorName, setValue):
        """
        This function takes in the name of an actuator and set its value.
        This will be replaced by a function in handlerSubsystem.py in the future.
        actuatorName (string): name of the actuator proposition.
        setValue     (bool)  : the value to set the proposition to.
        """

        self.motionControlGoToRegionWrapper(self.current_region, self.current_region)  # Stop, in case actuation takes time
        initial=False
        new_val = setValue #taken by the actuator argument, has to be called new_val
        exec(self.proj.actuator_handler[actuatorName])

    def motionControlGoToRegionWrapper (self,current_region, next_region):
        """
        This function wraps around the original goToRegion in motionControl handler. It takes in the current
        region object and the next region object, and at the end returns a boolean value of the arrival status
        to the next region.
        Inputs:
        current_region (region object): current region object
        next_region    (region object): next region object

        Ouputs:
        arrived (bool): status of arrival to the next region
        """
        current_region_no = self.proj.rfi.regions.index(current_region)
        next_region_no = self.proj.rfi.regions.index(next_region)
        arrived = self.proj.h_instance['motionControl'].gotoRegion(current_region_no, next_region_no)
        return arrived

