import fsa
import sys
import logging,random
import project
from collections import OrderedDict
import copy
import time

import lib.handlers.handlerTemplates as handlerTemplates

class ExecutorStrategyExtensions(object):
    """ Extensions to Executor to allow for the strategy structure.
        This class provides functions to update the outputs and to check for new states in every iteration.
    """

    def __init__(self):
        super(ExecutorStrategyExtensions, self).__init__()

        self.last_next_states= []
        self.last_next_state = None
        self.last_sensor_state = {}
        self.next_state      = None
        #self.current_region  = None
        #self.next_region     = None

        self.next_region                  = OrderedDict()
        self.current_region               = OrderedDict()
        self.next_region_heading          = {}
        self.prev_decomposed_region_names = {}
        self.prev_sensor_state            = {}

        self.noChangeOccured = 0

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

            # for fastslow
            key_removedRobot = key
            for robot in self.hsub.executing_config.robots:
                key_removedRobot = key_removedRobot.replace(robot.name+"_",'')
            if key_removedRobot in self.proj.regionMapping: continue

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
        # find current region
        self.current_region = self.strategy.current_state.getPropValue('region')

        # Take a snapshot of our current sensor readings
        sensor_state = self.hsub.getSensorValue(self.proj.enabled_sensors)

        # Let's try to transition
        # TODO: set current state so that we don't need to call from_state
        next_states = self.strategy.findTransitionableStates(sensor_state, from_state= self.strategy.current_state)

        # Make sure we have somewhere to go
        if len(next_states) == 0:
            # Well darn!
            logging.error("Could not find a suitable state to transition to!")
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
            self.next_region = self.next_state.getPropValue('region')

            self.postEvent("INFO", "Currently pursuing goal #{}".format(self.next_state.goal_id))

            # See what we, as the system, need to do to get to this new state
            self.transition_contains_motion = self.next_region is not None and (self.next_region != self.current_region)

            if self.transition_contains_motion:
                # We're going to a new region
                self.postEvent("INFO", "Heading to region %s..." % self.next_region.name)

            self.arrived = False

        if not self.arrived:
            # Move one step towards the next region (or stay in the same region)
            self.arrived = self.hsub.gotoRegion(self.current_region, self.next_region)

        # Check for completion of motion
        if self.arrived and self.next_state != self.strategy.current_state:
            # TODO: Check to see whether actually inside next region that we expected
            if self.transition_contains_motion:
                self.postEvent("INFO", "Crossed border from %s to %s!" % (self.current_region.name, self.next_region.name))

            if not self.proj.compile_options['fastslow']:
                # Run actuators after motion
                self.updateOutputs(self.next_state)

            self.strategy.current_state = self.next_state
            self.last_next_states = []  # reset

            self.postEvent("INFO", "Now in state %s (z = %s)" % (self.strategy.current_state.state_id, self.strategy.current_state.goal_id))

    def runStrategyIterationInstanteousAction(self):
        """
        Run, run, run the strategy with instantanous action!  (For one evaluation step)
        """
        # Take a snapshot of our current sensor readings
        sensor_state = self.hsub.getSensorValue(self.proj.enabled_sensors)

        ############################################################################
        ###############   not saving regionCompleted now ###########################
        ############################################################################
        decomposed_region_names = {}
        for robot in self.hsub.executing_config.robots:

            sensor_region = dict((k,v) for k, v in  sensor_state.iteritems() if (k.startswith(robot.name+"_") and k.endswith('_rc')))
            #logging.debug(robot.name + ": "+str(sensor_region))
            sensor_region_names = [k for k, v in  sensor_region.iteritems() if v]

            if len(sensor_region_names) == 1:
                sensor_region_name = sensor_region_names[0].replace('_rc','').replace(robot.name+"_",'')
                decomposed_region_names[robot.name] = self.proj.regionMapping[sensor_region_name]
                self.current_region[robot.name] = self.proj.rfi.regions[self.proj.rfi.indexOfRegionWithName(decomposed_region_names[robot.name][0])]

            else:
                # maybe we should change the inputs as well? or just wait for next iteration
                logging.info('not inside any region!')
                sensor_state             = self.prev_sensor_state
                decomposed_region_names  = self.prev_decomposed_region_names
                break

        """
        # the current region comes from strategy.current_state
        currentInputs = self.strategy.current_state.getInputs()
        decomposed_current_region_names = {}
        for robot in self.hsub.executing_config.robots:
            heading_region = dict((k,v) for k, v in  currentInputs.iteritems() if k.startswith(robot.name+"_") and k.replace(robot.name+"_",'').replace('_rc','') in self.proj.regionMapping)
            heading_region_names = [k for k, v in  heading_region.iteritems() if v]
            logging.debug("heading_region_names:" + str(heading_region_names))
            if heading_region_names:
                heading_region_name = heading_region_names[0].replace(robot.name+"_",'').replace('_rc','')
                decomposed_current_region_names[robot.name] = self.proj.regionMapping[heading_region_name]
                logging.debug("decomposed_current_region_names:" + str(decomposed_current_region_names))
                self.current_region[robot.name] = self.proj.rfi.regions[self.proj.rfi.indexOfRegionWithName(decomposed_current_region_names[robot.name][0])]

        """
        self.prev_decomposed_region_names = decomposed_region_names
        self.prev_sensor_state            =sensor_state
        logging.debug("self.current_region:" + str(self.current_region))

        ##############################################################################

        # Let's try to transition
        # TODO: set current state so that we don't need to call from_state
        next_states = self.strategy.findTransitionableStates(sensor_state, from_state= self.strategy.current_state)

        # Make sure we have somewhere to go
        if len(next_states) == 0:
            # Well darn!
            logging.error("Could not find a suitable state to transition to!")
            return

        if next_states != self.last_next_states:
            # NOTE: The last_next_states comparison is also to make sure we don't
            # choose a different random next-state each time, in the case of multiple choices
            self.last_next_states = next_states

            # Only allow self-transitions if that is the only option!
            #if len(next_states) > 1 and self.strategy.current_state in next_states:
            #    next_states.remove(self.strategy.current_state)

            # biasing towards the same last next_state and keeping the current state as an option
            # make sure we are only randomly picking when the set of next_states changed
            if self.last_next_state in next_states:
                self.next_state = self.last_next_state
            else:
                self.next_state = random.choice(next_states)
                self.last_next_state = self.next_state
            #self.next_state = random.choice(next_states)

            # find next region
            #############################################################################
            ############## not looking self.strategy.getPropValue('region')##############
            #############################################################################
            currentOutputs = self.strategy.current_state.getOutputs()
            decomposed_heading_region_names = {}
            for robot in self.hsub.executing_config.robots:
                heading_region = dict((k,v) for k, v in  currentOutputs.iteritems() if k.startswith(robot.name+"_") and k.replace(robot.name+"_",'') in self.proj.regionMapping)
                heading_region_names = [k for k, v in  heading_region.iteritems() if v]
                if heading_region_names:
                    heading_region_name = heading_region_names[0].replace(robot.name+"_",'')
                    decomposed_heading_region_names[robot.name] = self.proj.regionMapping[heading_region_name]
                    self.next_region[robot.name] = self.proj.rfi.regions[self.proj.rfi.indexOfRegionWithName(decomposed_heading_region_names[robot.name][0])]
                else:
                    #TODO: This is a hack. bdd is giving no next region...
                    self.next_region[robot.name] = self.current_region[robot.name]

            logging.debug("decomposed_heading_region_names" + str(decomposed_heading_region_names))
            #################################################################################
            self.postEvent("INFO", "Currently pursuing goal #{}".format(self.next_state.goal_id))

            ##################################################################################
            ################ not looking at self.next_state.getPropValue('regionCompleted') ##
            ##################################################################################
            nextOutputs = self.next_state.getOutputs()
            decomposed_heading_region_names = {}
            for robot in self.hsub.executing_config.robots:
                heading_region = dict((k,v) for k, v in  nextOutputs.iteritems() if k.startswith(robot.name+"_") and k.replace(robot.name+"_",'') in self.proj.regionMapping)
                heading_region_names = [k for k, v in  heading_region.iteritems() if v]
                if heading_region_names:
                    heading_region_name = heading_region_names[0].replace(robot.name+"_",'')
                    decomposed_heading_region_names[robot.name] = self.proj.regionMapping[heading_region_name]
                    self.next_region_heading[robot.name] = self.proj.rfi.regions[self.proj.rfi.indexOfRegionWithName(decomposed_heading_region_names[robot.name][0])]

            #logging.debug("next_input_region_names"+ str(next_input_region_names))
            ##################################################################################

            # See what we, as the system, need to do to get to this new state
            self.transition_contains_motion = None not in self.next_region.values() and True in  [self.next_region[robot.name] != self.current_region[robot.name] for robot in self.hsub.executing_config.robots]

            # Run actuators before motion
            self.updateOutputs(self.next_state)
            self.arrived = True
            self.last_sensor_state = sensor_state

        if not self.arrived or (self.next_state == self.strategy.current_state): # not sure what will happen
            # Move one step towards the next region (or stay in the same region)
            self.arrived = self.hsub.gotoRegionMultiRobot(self.current_region, self.next_region)
        # else:
            #TODO: temp fix for BDD
        #    self.hsub.gotoRegionMultiRobot(self.current_region, self.next_region)

        # Check for completion of motion
        if self.arrived and self.next_state != self.strategy.current_state:
            if self.transition_contains_motion:
                for robot in self.hsub.executing_config.robots:
                    if self.current_region[robot.name] != self.next_region[robot.name]:
                        if not isinstance(self.current_region[robot.name], int):
                            self.current_region[robot.name] = self.proj.rfi.regions.index(self.current_region[robot.name])
                        if not isinstance(self.next_region[robot.name], int):
                            self.next_region[robot.name] = self.proj.rfi.regions.index(self.next_region[robot.name])
                        self.postEvent("INFO", "Crossed border from %s to %s!(%s)" % (self.proj.rfi.regions[self.current_region[robot.name]].name, self.proj.rfi.regions[self.next_region[robot.name]].name, robot.name))
                        self.postEvent("INFO", "Heading to region %s...(%s)" % (self.next_region_heading[robot.name].name, robot.name))
            logging.info('*****************CHANGING STATES****************************')
            # stop robots
            self.robotList = [robot.name for robot in self.hsub.executing_config.robots]
            for robot_name in self.robotList:
                self.hsub.getHandlerInstanceByType(handlerTemplates.DriveHandler, robot_name).setVelocity(0, 0)
            self.noChangeOccured += 1
            logging.info("No of changed states:" + str(self.noChangeOccured))
            trueProp = []
            for prop, value in self.strategy.current_state.getAll().iteritems():
                if value:
                    trueProp.append(prop)
            logging.info("currentState:" + str(trueProp))
            trueProp = []
            for prop,value in self.next_state.getAll().iteritems():
                if value:
                    trueProp.append(prop)
            logging.info("nextState:" + str(trueProp))
            logging.info('************************************************************')
            self.strategy.current_state = self.next_state
            self.last_next_states = []  # reset
            if not self.usingSlugs:
                self.postEvent("INFO", "Now in state %s (z = %s)" % (self.strategy.current_state.state_id, self.strategy.current_state.goal_id))

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

