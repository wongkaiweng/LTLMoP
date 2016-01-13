import itertools
import project
import re
import fsa
import logging
from LTLParser.LTLFormula import LTLFormula, LTLFormulaType
from createJTLVinput import createLTLfile
import specCompiler
import threading
######### ENV Assumption Mining######
import specEditor
import livenessEditor
import wx
import time, sys, math
import LTLParser.LTLcheck
import parseEnglishToLTL
import numpy
import executeStrategy
######################################
# -------- two_robot_negotiation ----#
import logging
import copy
# -----------------------------------#
# ******** patching ********* #
import os
import LTLParser.translateFromSlugsLTLFormatToLTLFormat
import sets # make sure there's no duplicates of elements
import LTLParser.LTLRegion # for adding region parentheses
# *************************** #

class ExecutorResynthesisExtensions(object):
    """ Extensions to Executor to allow for specification rewriting and resynthesis.
        This class is not meant to be instantiated. """

    def __init__(self):
        super(ExecutorResynthesisExtensions, self).__init__()
        logging.info("Initializing resynthesis extensions...")

        # `next_proj` will store the temporary project we modify leading up to resynthesis
        self.next_proj = None

        # `needs_resynthesis` is an internal flag controlled by a resynthesis actuator handler
        # We don't call resynthesis code directly from the handler because we need to ensure
        # that it is only called after completion of all other actuators when transitioning to
        # a state that triggers resynthesis
        self.needs_resynthesis = False

        # Internal variables for receiving asynchronous user query responses
        self.user_query_response = ""
        self.received_user_query_response = threading.Event()

        ####################################################################################
        # HACK: Wrap the fsa runIteration function to check for internal flags at each step.
        # (Alternatively, this could be done with handler mapping manipulations or we could
        # put conditional code inside executor but this is least intrusive for now)
        ####################################################################################

        # We need to assign the functions to variables to make the
        # following closure work properly
        #original_fsa_runiteration = executeStrategy.runStrategyIteration#fsa.Automaton.runIteration
        #check_flags_func = self._checkForNewInternalFlags
        #def runIterationWithResynthesisChecks(self, *args, **kwds):
        #    """ Check for internal flags after every FSA runIteration() call """
        #    original_fsa_runiteration(self, *args, **kwds)
        #    check_flags_func()

        # Update the FSA function to point to our new one
        #executeStrategy.runStrategyIteration = runIterationWithResynthesisChecks
        
        # ---two_robot_negotiation ------ #
        self.exchangedSpec = False #track if we have exchanged spec with the other robot
        self.disableEnvChar = None #track if we are prioritized. False: env Characterization enabled. True: env Char disabled. None: no negotiation
        self.lastSensorState  = None
        self.negotiationStatus= None #track who is appending spec
        self.receivedSpec = False #track if we have recevied request from the other robot
        self.sentSpec = False #track if we have sent spec to the other robot
        self.violationTimeStamp = 0
        # ------------------------------- #

    def _checkForNewInternalFlags(self):
        """ Detect whether any "internal flags" (i.e. propositions beginning with an underscore)
            have become true in the last timestep, or resynthesis is pending; if so, perform an
            action based on this. """

        ### Look for rising, underscore-prefixed propositions ###

        # NOTE: This needs to be done /before/ resynthesis to ensure we are using the fully
        #       rewritten specification at resynthesis time

        # Check the current state of the automaton
        currently_true_props = set((p for p, v in self.aut.current_outputs.iteritems() if int(v) == 1))

        # Flags can't go true in the first time-step
        if not hasattr(self.aut, "previously_true_props"):
            self.aut.previously_true_props = currently_true_props
            return

        # Look for rising, underscore-prefixed propositions and process them
        rising_propositions = currently_true_props - self.aut.previously_true_props
        for p in rising_propositions:
            if p.startswith("_"):
                self._processInternalFlag(p)

        self.aut.previously_true_props = currently_true_props

        ### See if the resynthesis actuator handler has let us know we need to resynth ###

        if self.needs_resynthesis:
            if self.next_proj is not None:
                self.resynthesizeFromProject(self.next_proj)
            else:
                logging.error("Resynthesis was triggered before any spec rewrites.  Skipping.")

            # Clear the resynthesis flag
            self.needs_resynthesis = False

    def _findGroupsInCorrespondenceWithGroup(self, group_name):
        """ Return a list of group names to which the group named `group_name` has a correspondence relation. """
        
        # TODO: Move this to parser?
        CorrespondenceDefinitionRE = re.compile(r"^\s*" + group_name + r"\s+corresponds?\s+to\s+(?P<groupB>\w+)", \
                                                re.MULTILINE | re.IGNORECASE)
        
        corresponding_groups = [m.group("groupB") for m in CorrespondenceDefinitionRE.finditer(self.proj.specText)]
        
        return corresponding_groups

    def _processInternalFlag(self, flag_name):
        """ Respond appropriately to an "internal flag" proposition having been triggered.
            Note that this is only called on rising edges. """

        #################################################
        ### Check for group modification propositions ###
        #################################################

        # Use a regex on the proposition name to figure out what we should do
        m = re.match(r"_(?P<action>add_to|remove_from)_(?P<groupName>\w+)", \
                     flag_name, re.IGNORECASE)

        # We currently only handle this one type of flag, so there's nothing to do
        # if it doesn't match
        if m is None:
            return

        # Create next_proj if we haven't yet
        # This is what we'll be working on, and then eventually resynthesizing from
        if self.next_proj is None:
            self.next_proj = self._duplicateProject(self.proj)

        # We've been told to add or remove something from a group, but we need to figure
        # out exactly what that /something/ is (the "referent").  In some cases, this could probably
        # be resolved automatically using certain heuristics, but for now we will explicitly
        # ask the user what to do.

        response = self.queryUser("What should I add to the group {!r}?".format(m.group("groupName")))

        # Cast the referent to a list (there may be more than one in the case of new region detection)
        referents = [response]

        logging.debug("Resolved referents to {}.".format(referents))

        if m.group('action').lower() == "add_to":
            logging.info("Added item(s) %s to group %s.", ", ".join(referents), m.group('groupName'))

            # Rewrite the group definition in the specification text
            self._updateSpecGroup(m.group('groupName'), 'add', referents)

            # Get a list of region propositions
            region_names = [r.name for r in self.proj.rfi.regions]
            if self.proj.compile_options["decompose"]: # :((((
                region_names += [r.name for r in self.proj.rfiold.regions]

            # Add any new propositions to the project's proposition lists
            for ref in referents:
                if ref in self.next_proj.enabled_sensors + region_names:
                    # This prop already exists; don't need to add it.
                    continue

                # Assume all new user-added props are sensors for now.
                logging.debug("Adding new sensor proposition {!r}".format(ref))
                self.next_proj.enabled_sensors.append(ref)
                self.next_proj.all_sensors.append(ref)
            
            # Figure out if there are any corresponding groups which we will also need to update
            corresponding_groups = self._findGroupsInCorrespondenceWithGroup(m.group('groupName'))
            logging.debug("Need to also update corresponding groups: {}".format(corresponding_groups))

            # Process each corresponding group
            for corr_group_name in corresponding_groups:
                # Suggest a default name
                new_prop_name_default = corr_group_name + "_" + referents[0]

                # Ask for user input
                new_prop_name = self.queryUser("Name for new proposition in group {} that corresponds to {}?".format(corr_group_name, referents[0]), \
                                               default_response=new_prop_name_default)

                # Rewrite the spec
                self._updateSpecGroup(corr_group_name, 'add', [new_prop_name])
                logging.info("Added corresponding item %s to group %s.", new_prop_name, corr_group_name)

                # Assume the new prop is either a region or an actuator, and add to actuators if necessary
                if new_prop_name not in self.next_proj.enabled_actuators + self.next_proj.all_customs + region_names:
                    self.next_proj.enabled_actuators.append(new_prop_name)
                    self.next_proj.all_actuators.append(new_prop_name)
                    logging.debug("Adding new actuator proposition {!r}".format(new_prop_name))

        elif m.group('action').lower() == "remove_from":
            # TODO: Removal from groups has not been tested and is likely not to work correctly

            # Rewrite the group definition in the specification text
            logging.info("Removed item(s) %s from group %s.", ", ".join(referents), m.group('groupName'))
            self._updateSpecGroup(m.group('groupName'), 'remove', referents)

    def _updateSpecGroup(self, group_name, operator, operand):
        """ Rewrite the text of the specification in `self.next_proj` by modifying proposition
            group `group_name` with operator `operator` (e.g. "add"/"remove") and
            operand `operand` (a list of propositions) """

        # Make a regex for finding the group definition in the spec
        PropositionGroupingRE = re.compile(r"^group\s+%s\s+(is|are)\s+(?P<propositions>.+)\n" % group_name, \
                                      re.IGNORECASE|re.MULTILINE)

        # Define a function for modifying the list of propositions in-place
        def gen_replacement_text(group_name, operand, m):
            """ Given a group name, modification operand, and match object from
                PropositionGroupingRE, return a new group definition line with an appropriately
                updated list of propositions."""

            # Figure out what propositions are already there
            propositions = re.split(r"\s*,\s*", m.group('propositions'))

            # Remove the "empty" placeholder if it exists
            propositions = [p for p in propositions if p != "empty"]

            # Perform the operation on the list of propositions
            if operator == "add":
                propositions.extend(operand)
            elif operator == "remove":
                propositions = [p for p in propositions if p not in operand]
            else:
                logging.error("Unknown group modification operator {!r}".format(operator))

            # Re-add the "empty" placeholder if the result of the operation is now empty
            if not propositions:
                propositions = ["empty"]

            # Create a new group definition line
            new_group_definition = "group %s is %s\n" % (group_name, ", ".join(propositions))

            return new_group_definition

        self.next_proj.specText = PropositionGroupingRE.sub(lambda m: gen_replacement_text(group_name, operand, m), \
                                                            self.next_proj.specText)

    def _duplicateProject(self, proj, n=itertools.count(1)):
        """ Creates a copy of a proj, and creates an accompanying spec file with an
            auto-incremented counter in the name."""

        # reload from file instead of deepcopy because hsub stuff can
        # include uncopyable thread locks, etc
        # TODO: fix this ^
        new_proj = project.Project()
        new_proj.setSilent(True)
        new_proj.loadProject(proj.getFilenamePrefix() + ".spec")

        # copy hsub references manually
        new_proj.hsub = proj.hsub
        new_proj.hsub.proj = new_proj # oh my god, guys
        new_proj.h_instance = proj.h_instance
        new_proj.rfiold = proj.rfiold

        new_proj.sensor_handler = proj.sensor_handler
        new_proj.actuator_handler = proj.actuator_handler

        # Choose a name by incrementing the stepX suffix
        # NOTE: old files from previous executions will be overwritten

        # Take the current proj name and remove any "stepX" part
        base_name = self.proj.getFilenamePrefix().rsplit('.', 1)[0]

        # Add a new "stepX" part
        newSpecName = "%s.step%d.spec" % (base_name, n.next())

        # Save the file
        new_proj.writeSpecFile(newSpecName)

        logging.info("Created new spec file: %s", newSpecName)

        return new_proj

    def _setSpecificationInitialConditionsToCurrent(self, proj):
        """ Remove any existing initial conditions from the guarantees portion of the
            LTL specification and replace them with the current state of the system.

            TODO: Propositions that don't exist in both old and new specifications are
            ignored in the process?"""

        # TODO: support doing this at the language level too?
        # TODO: what if state changes during resynthesis? should we be less restrictive?

        # Parse the LTL file in so we can manipulate it
        ltl_filename = proj.getFilenamePrefix() + ".ltl"
        assumptions, guarantees = LTLFormula.fromLTLFile(ltl_filename)

        # Get a conjunct expressing the current state
        ltl_current_state = self.getCurrentStateAsLTL() # TODO: Constrain to props in new spec

        logging.debug("Constraining new initial conditions to: " + ltl_current_state)

        # TODO: Do we need to remove pre-exisiting constraints too? What about env?
        # Add in current system state to make strategy smaller
        gc = guarantees.getConjuncts()
        
        if ltl_current_state != "":
            gc.append(LTLFormula.fromString(ltl_current_state))  

        # Write the file back
        createLTLfile(ltl_filename, assumptions, gc)

    def resynthesizeFromProject(self, new_proj):
        """ Given a new project `new_proj`, pause execution, synthesize this new project,
            swap it in for the old project, and then resume execution. """

        # TODO: reload from file less often

        self.pause()

        self.postEvent("INFO", "Starting resynthesis. Please wait...")

        # Save the file
        new_proj.writeSpecFile()

        # Get a SpecCompiler ready
        c = specCompiler.SpecCompiler()
        c.proj = new_proj

        # Make sure rfi is non-decomposed here
        c.proj.loadRegionFile(decomposed=False)

        if c.proj.compile_options["decompose"]:
            c._decompose()

        # Call the parser
        c._writeLTLFile()
        c._writeSMVFile()

        # Constrain the initial conditions to our current state
        self._setSpecificationInitialConditionsToCurrent(new_proj)

        # Synthesize a strategy
        (realizable, realizableFS, output) = c._synthesize()
        logging.debug(output)

        # Check if synthesis succeeded
        if not (realizable or realizableFS):
            logging.error("Specification for resynthesis was unsynthesizable!")
            self.postEvent("INFO", "ERROR: Resynthesis failed.  Please check the terminal log for more information.")
            self.pause()
            return False

        logging.info("New automaton has been created.")

        # Load in the new strategy

        self.proj = new_proj

        logging.info("Reinitializing execution...")

        spec_file = self.proj.getFilenamePrefix() + ".spec"
        aut_file = self.proj.getFilenamePrefix() + ".aut"
        self.initialize(spec_file, aut_file, firstRun=False)

        # Clear next_proj again
        self.next_proj = None

        self.postEvent("INFO", "Resynthesis complete.  Resuming execution.")
        self.resume()

        return True

    def resynthesizeFromNewSpecification(self, spec_text):
        """ Given a text string of a new specification, resynthesize with a copy of the current
            project after swapping in the next `spec_text` for the previous specification. """

        # Copy the current project
        new_proj = self._duplicateProject(self.proj)

        # Overwrite the specification text
        new_proj.specText = spec_text
        
        return resynthesizeFromProject(new_proj)

    ########### ENV Assumption mining ###############
    ### added an option for env inputs ##############
    def getCurrentStateAsLTL(self, env_output=False):
        """ Return a boolean formula (as a string) capturing the current discrete state of 
            the system (and, optionally, the environment as well) """

        if self.aut:
            # If we have current state in the automaton, use it (since it can capture
            # state of internal propositions).
            return fsa.stateToLTL(self.aut.current_state, env_output=env_output)
        else:
            # If we have no automaton yet, determine our state manually
            # NOTE: This is only relevant for dialogue-based specifications
            # TODO: Support env
            # TODO: Look at self.proj.currentConfig.initial_truths and pose
            return ""
    ###################################################
    def queryUser(self, question, default_response="", accept_empty_response=False):
        """ Ask the user for an input, prompting with `question`.  `default_response` will
            be provided as a recommended response.  If `accept_empty_response` is False,
            the question will be re-asked until the user provides a non-empty answer.  """

        # FIXME: This will have problems if a second query is issued before the first terminates

        self.user_query_response = None

        # If we aren't accepting empty responses, Keep asking the user until they give a
        # non-empty response
        while self.user_query_response is None or \
              self.user_query_response.strip() == "" and not accept_empty_response:

            # Delegate the query to whatever user interface is attached to execute (e.g. SimGUI)
            self.received_user_query_response.clear()
            self.postEvent("QUERY_USER", [question, default_response])

            # Block until we receive a response
            # WARNING: The controller will be unresponsive during this period.
            self.received_user_query_response.wait()

        return self.user_query_response

    def processUserQueryResponse(self, answer):
        """ Callback function to receive a response to a user query. """

        logging.debug("Got user query response {!r}".format(answer))

        # Save the response
        self.user_query_response = answer

        # Let anyone waiting for the response know that it is ready
        self.received_user_query_response.set()
   
    # --- two_robot_negotiation --- #
    def resetEnvCharacterization(self):
        """
        This function reset old values of violated things and replace LTLViolationCheck EnvTrans
        """
        # reset env characterization
        self.spec['EnvTrans'] = self.oriEnvTrans
        self.LTLViolationCheck.replaceLTLTree(self.oriEnvTrans)
        self.LTLViolationCheck.resetEnvCharacterization()
        self.LTLViolationCheck.setOriginalEnvTrans(self.spec['EnvTrans'].replace('[]',''))
        self.old_violated_specStr = []
        self.old_violated_specStr_with_no_specText_match = []
        self.old_possible_states_violated_specStr = []
        self.old_possible_states_violated_specStr_with_no_specText_match = []

    def synthesizeWithExchangedSpec(self, level = False):
        """
        spec negotiation in execute.py
        """
        self.envViolationThres = 1000    # set higher thres for recovery

        # fist append new environment /and system init state
        self._setSpecificationInitialConditionsToCurrentInDNF(self.proj,False, self.sensor_strategy)

        # obtain SysGoals, EnvTrans of the other robot 
        # may not have anything the other robot have not sent info. (dealt with inside requestSpec) -- may move the check here.
        otherRobotSysGoals = self.robClient.requestSpec('SysGoals')
        otherRobotEnvTrans = self.robClient.requestSpec('EnvTrans').replace('[]','\n\t\t\t[]')
        self.receivedSpec = True

        # see if we can take the other robot's actions into account. 
        # first we safe a copy of our SysTrans and EnvGoals before modification
        oldSpecSysTrans = self.spec['SysTrans']
        oldSpecEnvGoals = self.spec['EnvGoals']

        # conjunct the spec of the other robots
        self.spec['SysTrans'] = ' &\n'.join(filter(None, [otherRobotEnvTrans, oldSpecSysTrans]))
        #logging.debug('SysTrans:' + self.spec['SysTrans'])
        
        if level:
            # with only systrans
            self.spec['EnvGoals'] = oldSpecEnvGoals
        elif  "[]<>(TRUE)" in oldSpecEnvGoals or not oldSpecEnvGoals:
            self.spec['EnvGoals'] = otherRobotSysGoals
        else:
            self.spec['EnvGoals'] = otherRobotSysGoals + '&' +  oldSpecEnvGoals
        #logging.debug('EnvGoals:' + self.spec['EnvGoals'])
        
        # resynthesize
        self.postEvent("NEGO","Use exchanged information to synthesize new controller.")
        self.recreateLTLfile(self.proj)
        realizable, _, _ = self.compiler._synthesize()

        return realizable, oldSpecSysTrans, oldSpecEnvGoals

    def appendSpecFromEnvRobots(self):
        """
        asked for spec from negotiation monitor and append to our spec.
        """

        # make resynthesis with recovery
        #self.recovery = True
        #self.proj.compile_options['recovery'] = True
        #self.compiler.proj.compile_options['recovery'] = True

        # modify envTrans to remove all characterization. Restart Characterization
        self.resetEnvCharacterization() # reset env characterization both ways

        self.postEvent('NEGO','-- NEGOTIATION STARTED --')
        # send SysGoals, EnvTrans and EnvGoals
        self.robClient.sendSpec('SysGoals',self.spec['SysGoals'], self.proj.compile_options["fastslow"], self.proj.compile_options["include_heading"])
        if self.proj.compile_options["fastslow"]:
            self.robClient.sendSpec('EnvTrans', LTLParser.LTLcheck.removeLTLwithoutKeyFromEnvTrans(self.oriEnvTrans, self.proj.otherRobot[0]), self.proj.compile_options["fastslow"], self.proj.compile_options["include_heading"])
        else:
            self.robClient.sendSpec('EnvTrans',self.spec['EnvTrans'])
        #self.robClient.sendSpec('EnvGoals',self.spec['EnvGoals'])
        self.sentSpec = True

        self.robClient.setNegotiationStatus("'" + self.proj.otherRobot[0] + "'")

        # here we have an extra step to synthesize with exchanged spec too
        self.postEvent('NEGO','Both of us try to incorporate the requirements of the other.')
        self.postEvent("NEGO",'Adding system guarantees with environment goals.')
        realizable, _, _ = self.synthesizeWithExchangedSpec(False)

        while self.robClient.checkNegotiationStatus() == self.proj.otherRobot[0]:
            # wait for the actions of the other robot
            time.sleep(2)

        if self.robClient.checkNegotiationStatus() == True:
            if realizable:
                self.disableEnvChar = False # env characterization enabled
                self.postEvent('NEGO','Incorporated the other robot''s requirements. The other robot has incorporated ours.')
                self.postEvent('NEGO','-- NEGOTIATION ENDED --')
                self.postEvent('RESOLVED','')
            else:
                # we need to use our original spec
                self.postEvent('NEGO','CANNOT incororate the other robot'' requirements.')
                self.postEvent('NEGO','Ask the other robot to include our actions in its controller.')
                self.spec = copy.deepcopy(self.originalSpec)
                self.spec['EnvTrans'] = self.oriEnvTrans
                #convert to the original specification
                self._setSpecificationInitialConditionsToCurrentInDNF(self.proj,False, self.sensor_strategy)
                self.recreateLTLfile(self.proj, spec = self.spec)
                realizable, _, _ = self.compiler._synthesize()
                logging.debug("Original spec with new env/sys init realizable?" + str(realizable))
                self.disableEnvChar = True # env characterization disabled
                self.postEvent('NEGO','The other robot has incorporated our actions. We will use the original spec')
                self.postEvent('NEGO','-- NEGOTIATION ENDED --')
                self.postEvent('RESOLVED','')

        elif self.robClient.checkNegotiationStatus() == self.robClient.robotName: #and not self.receivedSpec:
            self.postEvent('NEGO','The other robot cannot incorporate our actions.')# We will try incorporating its actions instead.')
            # try to synthesize controller with spec from the other robot instead
            ###### ONE STEP ####
            #realizable, _, _ = self.synthesizeWithExchangedSpec(False)

            if realizable:
                self.postEvent('NEGO','We can continue with the exchanged information.')
                self.postEvent('NEGO','-- NEGOTIATION ENDED --')
                self.postEvent('RESOLVED','')
                self.robClient.setNegotiationStatus(True)
                self.disableEnvChar = False # env characterization enabled
            else:
                self.postEvent('NEGO','Negotiation failed')
                self.postEvent('NEGO','-- NEGOTIATION ENDED --')
                self.robClient.setNegotiationStatus(False)
                self.disableEnvChar = False # env characterization enabled

        else: #We have received spec before and it's unrealizable
            realizable = False
            self.postEvent('NEGO','Negotiation failed')
            self.postEvent('NEGO','-- NEGOTIATION ENDED --')
            self.robClient.setNegotiationStatus(False)
            self.disableEnvChar = False # env characterization enabled

        # reset violtion timestamp
        self.violationTimeStamp = 0
        self.robClient.setViolationTimeStamp(self.violationTimeStamp)
        logging.debug('Resetting violation timeStamp')
        time.sleep(1)

        return realizable

    def receiveRequestFromEnvRobot(self):
        """
        check if a request for negotiation is received.
        Returns true if negotiation occurs and false otherwise
        """
        # resynthesis request from the other robot
        self.negotiationStatus = self.robClient.checkNegotiationStatus()
        if (not self.exchangedSpec or not self.receivedSpec) and self.negotiationStatus == self.robClient.robotName:

            # also reset env characterization
            self.resetEnvCharacterization() # reset env characterization both ways

            # prepare for conservative step of the sender (send him/her specs)
            # send our spec to the other robot
            self.robClient.sendSpec('SysGoals',self.spec['SysGoals'], self.proj.compile_options["fastslow"], self.proj.compile_options["include_heading"])
            if self.proj.compile_options["fastslow"]:
                self.robClient.sendSpec('EnvTrans', LTLParser.LTLcheck.removeLTLwithoutKeyFromEnvTrans(self.oriEnvTrans, self.proj.otherRobot[0]), self.proj.compile_options["fastslow"], self.proj.compile_options["include_heading"])
            else:
                self.robClient.sendSpec('EnvTrans',self.spec['EnvTrans'])
            #self.robClient.sendSpec('EnvGoals',self.spec['EnvGoals'])

            # make resynthesis with recovery
            #self.recovery = True
            #self.proj.compile_options['recovery'] = True
            #self.compiler.proj.compile_options['recovery'] = True

            self.postEvent('NEGO','-- NEGOTIATION STARTED --')
            self.postEvent('NEGO','Received request to coordinate.')
            self.postEvent('NEGO','Both of us try to incorporate the requirements of the other.')
            self.postEvent("NEGO",'Adding system guarantees with environment goals.')
            # synthesize a new controller to incorporate the actions of the other robot.
            ###### ONE STEP ######
            realizable, _, _ = self.synthesizeWithExchangedSpec(False)

            if realizable:
                self.robClient.setNegotiationStatus(True)
                self.disableEnvChar = False
                self.postEvent('NEGO','Using exchanged specification.')
                self.postEvent('NEGO','-- NEGOTIATION ENDED --')
                self.postEvent('RESOLVED','')

                # reset violtion timestamp
                self.violationTimeStamp = 0
                self.robClient.setViolationTimeStamp(self.violationTimeStamp)
                logging.debug('Resetting violation timeStamp')
                time.sleep(1)

                self.sentSpec = True
                self.exchangedSpec = True
                # reinitialize automaton
                spec_file = self.proj.getFilenamePrefix() + ".spec"
                aut_file = self.proj.getFilenamePrefix() + ".aut"
                self.initialize(spec_file, aut_file, firstRun=False)

            elif not self.sentSpec:
                self.postEvent('NEGO','Unrealizable with exchanged info. Asking the other robot to incorporate our actions instead.')
                self.sentSpec = True
                self.robClient.setNegotiationStatus("'" + self.proj.otherRobot[0] + "'")

                # wait until the other robot resynthesize its controller
                while not isinstance(self.robClient.checkNegotiationStatus(), bool): #self.robClient.checkNegotiationStatus() != (True or False):
                    time.sleep(2)

                if self.robClient.checkNegotiationStatus() == True:
                    # remove spec from other robots and resynthesize
                    self.spec = copy.deepcopy(self.originalSpec)
                    self.spec['EnvTrans'] = self.oriEnvTrans

                    #convert to the original specification
                    self._setSpecificationInitialConditionsToCurrentInDNF(self.proj,False, self.sensor_strategy)
                    self.recreateLTLfile(self.proj, spec = self.spec)
                    realizable, _, _ = self.compiler._synthesize()
                    logging.debug("Original spec with new env/sys init realizable?" + str(realizable))
                    self.disableEnvChar = True # env characterization disabled
                    self.postEvent('NEGO','The other robot has incorporated our action. Using original specification.')
                    self.postEvent('NEGO','-- NEGOTIATION ENDED --')
                    self.postEvent('RESOLVED','')

                    # reset violtion timestamp
                    self.violationTimeStamp = 0
                    self.robClient.setViolationTimeStamp(self.violationTimeStamp)
                    logging.debug('Resetting violation timeStamp')
                    time.sleep(1)

                else:
                    #TODO: spec analysis needed.
                    self.postEvent('NEGO','Negotiation Failed. Spec Analysis is needed.')
                    self.postEvent('NEGO','-- NEGOTIATION ENDED --')
                    self.robClient.setNegotiationStatus(False)
                    sys.exit()
                    #self.onMenuAnalyze(enableResynthesis = False, exportSpecification = True)

            else: # sent spec before
                #TODO: spec analysis needed.
                self.postEvent('NEGO','Negotiation Failed. Spec Analysis is needed.')
                self.postEvent('NEGO','-- NEGOTIATION ENDED --')
                self.robClient.setNegotiationStatus(False)
                sys.exit()
                #self.onMenuAnalyze(enableResynthesis = False, exportSpecification = True)

            self.exchangedSpec = True
            return True
        else:
            return False
    # ------------------------------# 
    
    ########### ENV Assumption Mining ####################
    def addStatetoEnvSafety(self, sensor_state, firstRun=False, checker=None):
        """
        append the current detected state to our safety
        """
        if checker is None:
            checker = self.LTLViolationCheck

        realizable=True
        if (self.disableEnvChar is None) or (not self.disableEnvChar):
            # Add the current state in init state of the LTL spec
            self.postEvent("VIOLATION","Adding the current state to our initial conditions")
            self._setSpecificationInitialConditionsToCurrentInDNF(self.proj,firstRun, sensor_state)

            # ------ patching ---------#
            # just update initial condition
            if self.proj.compile_options['neighbour_robot'] and self.proj.compile_options["multi_robot_mode"] == "patching":
                self.recreateLTLfile(self.proj)
                realizable, _, _ = self.compiler._synthesize()  # TRUE for realizable, FALSE for unrealizable
                self.postEvent("VIOLATION", "The specification is " + ("realizable." if realizable else "unrealizable."))
            # -------------------------#

            if not( self.proj.compile_options['neighbour_robot'] and self.proj.compile_options["multi_robot_mode"] == "patching") or realizable is False:
                if not firstRun:
                    self.spec['EnvTrans'] = checker.modify_LTL_file("")

                self.recreateLTLfile(self.proj)
                realizable, _, _ = self.compiler._synthesize()  # TRUE for realizable, FALSE for unrealizable
             
                if not firstRun:
                    self.postEvent("VIOLATION",self.simGUILearningDialog[checker.modify_stage-1] + " and the specification is " + ("realizable." if realizable else "unrealizable."))
                    if not realizable:
                        while checker.modify_stage < 2 and not realizable:   # < 3 and not realizable:        # CHANGED TO 2 FOR PAPER

                            checker.modify_stage += 1
                            self.spec['EnvTrans'] = checker.modify_LTL_file("")
                            self.recreateLTLfile(self.proj)
                            realizable, _, _  = self.compiler._synthesize()  # TRUE for realizable, FALSE for unrealizable

                            self.postEvent("VIOLATION",self.simGUILearningDialog[checker.modify_stage-1] + " and the specification is " + ("realizable." if realizable else "unrealizable."))

                        if self.proj.compile_options['neighbour_robot']:
                            if not realizable:
                                #realizable = False
                                if self.proj.compile_options["multi_robot_mode"] == "negotiation":
                                    # ------ two_robot_negotiation  -------- #
                                    # see if the other robot has violation before us
                                    otherRobotViolationTimeStamp = self.robClient.getViolationTimeStamp(self.proj.otherRobot[0])
                                    logging.debug("otherRobotViolationTimeStamp:" + str(otherRobotViolationTimeStamp))
                                    logging.debug('self.violationTimeStamp:' + str(self.violationTimeStamp))

                                    # exchange info with the other robot and see if it is realizable.
                                    # later time can exchange spec
                                    if ((not self.exchangedSpec) or (not self.sentSpec and self.receivedSpec)) and otherRobotViolationTimeStamp < self.violationTimeStamp:
                                        # exchange spec
                                        realizable = self.appendSpecFromEnvRobots()
                                        self.exchangedSpec = True
                                    elif self.sentSpec and self.receivedSpec:
                                        pass
                                    else:
                                        return
                                    # -------------------------------------- #

                                elif self.proj.compile_options["multi_robot_mode"] == "patching":
                                    # ************ patching ****************** #
                                    # Take care of everything to start patching
                                    #self.postEvent("RESOLVED","")
                                    #self.postEvent("PATCH","We will now ask for a centralized strategy to be executed.")
                                    #self.initiatePatching()
                                    logging.error("Patching is now done in excute.py. We should not have got here!")
                                    # return and continue execution
                                    return
                                    # **************************************** #

                                else:
                                    logging.error("Mulit robot mode is incorrect. This is impossible.")

            # only update ltl_tree in LTLcheck if the spec is realizable
            if realizable:
                checker.updateEnvTransTree("")

        else:
            logging.debug('Waiting for the other robot to yield our way.')
            #logging.debug(self.lastSensorState.getInputs())
            #logging.debug(sensor_state.getInputs())

            # only append new init state if there's no request of negotiation
            if not self.receiveRequestFromEnvRobot():
                if self.lastSensorState is None  or self.lastSensorState.getInputs(expand_domains=True) != sensor_state.getInputs(expand_domains=True) or not realizable:
                    # update spec with current state of the other robot
                    self._setSpecificationInitialConditionsToCurrentInDNF(self.proj,firstRun, sensor_state)
                    self.recreateLTLfile(self.proj)
                    realizable, _, _ = self.compiler._synthesize()  # TRUE for realizable, FALSE for unrealizable
                    self.postEvent('INFO','Recreating automaton based on the new env init state.')

        self.lastSensorState = sensor_state   

        if not self.proj.compile_options['neighbour_robot'] or not self.proj.compile_options["multi_robot_mode"] == "d-patching":
            if not realizable:
                self.postEvent("VIOLATION","Please enter some environment liveness to make the specification realizable.")
                # Use Vasu's analysis tool
                self.onMenuAnalyze()  # in resynthesis.py
                realizable ,_ ,_ = self.compiler._synthesize()  # TRUE for realizable, FALSE for unrealizable

                if not realizable:
                    self.postEvent("VIOLATION", "Specification is still unrealizable after adding env liveness. Now we will exit the execution")
                    sys.exit()
                else:
                    checker.modify_stage  = 1 # reset the stage to 1


             
        # reload aut file if the new ltl is realizable                  
        if realizable or self.disableEnvChar:
            if not self.disableEnvChar:
                self.postEvent("RESOLVED", "The specification violation is resolved.")

                # clear all violations and resume runtime monitoring
                self.resumeRuntimeMonitoring()

                # ------------ two_robot_negotiation ----------#
                # store time stamp of violation
                if self.proj.compile_options['neighbour_robot'] and self.proj.compile_options["multi_robot_mode"] == "negotiation":
                    self.violationTimeStamp = 0
                    self.robClient.setViolationTimeStamp(self.violationTimeStamp)
                    logging.debug('Resetting violation timeStamp')
                # ---------------------------------------------- #
                #######################
                # Load automaton file #
                #######################
                self.postEvent("INFO","Reloading automaton and Initializing...")

            else:
                # wait for a while till we reload automaton
                time.sleep(2)
                
            spec_file = self.proj.getFilenamePrefix() + ".spec"
            aut_file = self.proj.getFilenamePrefix() + ".aut"    
            init_state, new_strategy = self.initialize(spec_file, aut_file, firstRun=False)  
            
        else:
            self.postEvent("VIOLATION", "Specification is still unrealizable. We will exit the execution")
            #self.onMenuAnalyze(enableResynthesis = False, exportSpecification = True)
            sys.exit()            
                
        return init_state, new_strategy
    
    def _setSpecificationInitialConditionsToCurrentInDNF(self, proj, firstRun, sensor_state):
        """ Add Env and Sys Init in disjunctive Normal form to LTL
        proj:     our current project
        firstRun: if this is the first time running simGUI
        **All region_b and regionCompleted_b have been converted to bit and sbit
        """
        ## for sensors
        current_env_init_state  = sensor_state.getLTLRepresentation(mark_players=True, use_next=False, include_inputs=True, include_outputs=False)

        ## for actuators and regions
        if firstRun or self.strategy is None:
            tempY = []
            # figure out our current region
            init_region = self._getCurrentRegionFromPose()
            for prop, value in sensor_state.context.expandDomainsInPropAssignment({'region':self.proj.rfi.regions[init_region]}).iteritems():
                tempY.append(prop.replace('region_b','s.bit') if value else '!'+prop.replace('region_b','s.bit'))

            # Figure out our initially true outputs
            init_outputs = []
            for prop in self.hsub.executing_config.initial_truths:
                if prop not in self.proj.enabled_sensors:
                    init_outputs.append(prop)
            
            for x in self.proj.enabled_actuators:
                if x not in init_outputs:
                    tempY.append("!s." + str(x))
                else:
                    tempY.append("s." + str(x))
                    
            for x in self.proj.all_customs:
                if x not in init_outputs:
                    tempY.append("!s." + str(x))
                else:
                    tempY.append("s." + str(x))
                    
            current_sys_init_state = " & ".join(tempY)

        else:
            # find the current inputs and outputs from strategy and replace region_b
            current_sys_init_state  = self.strategy.current_state.getLTLRepresentation(mark_players=True, use_next=False, include_inputs=False, include_outputs=True)

            # if using fastslow set env init bits to be the same as sys init bits
            if self.proj.compile_options['fastslow']:
                # iterate each bit
                for x in range(max(1, int(math.ceil(math.log(len(self.proj.rfi.regions), 2))))):
                    negate = False
                    if "!s.bit" + str(x) in current_sys_init_state:
                        negate = True

                    if "!e.sbit" + str(x) in current_env_init_state:
                        if not negate:
                            current_sys_init_state = current_sys_init_state.replace("s.bit" + str(x), "!s.bit" + str(x))
                    else:
                        if negate:
                            current_sys_init_state = current_sys_init_state.replace("!s.bit" + str(x), "s.bit" + str(x))

        # try to compare current spec with the new clause so that no duplicates are added
        cur_sys_init = "(" + current_env_init_state.replace("\t", "").replace("\n", "").replace(" ", "") + "&"+ current_sys_init_state.replace("\t", "").replace("\n", "").replace(" ", "") + ")"

        # connect the original sysInit with the current system init
        #self.spec["SysInit"]  = self.originalSysInit + "\n| " + cur_sys_init
        self.spec['EnvInit'] = "(" + current_env_init_state.replace("\t", "").replace("\n", "").replace(" ", "") + ")"
        self.spec["SysInit"] = "(" + current_sys_init_state.replace("\t", "").replace("\n", "").replace(" ", "") + ")"

    def recreateLTLfile(self, proj, spec = None , export = False):
        """
        rewrite the LTL file with the modified spec
        spec: passed in LTL specificaion by the user. Defaulted to be self.spec 
        """
        
        if spec == None:
            spec = self.spec
        
        if export:
            ltl_filename = proj.getFilenamePrefix() + "Generated.ltl"
        else:
            ltl_filename = proj.getFilenamePrefix() + ".ltl"       
        
        # putting all the LTL fragments together (see specCompiler.py to view details of these fragments)
        LTLspec_env = ' &\n'.join(filter(None,[spec['InitEnvRegionSanityCheck'], spec["EnvInit"], spec["EnvTrans"], spec["EnvGoals"]]))
        LTLspec_sys = ' &\n'.join(filter(None,[spec["SysInit"], spec["SysTrans"], spec["SysGoals"], spec['InitRegionSanityCheck'], spec['Topo']]))

        if proj.compile_options["fastslow"]:
            # EnvTopo is in EnvTrans already
            LTLspec_env = ' &\n'.join(filter(None,[LTLspec_env, spec['SysImplyEnv']]))

        # Write the file back
        createLTLfile(ltl_filename, LTLspec_env, LTLspec_sys)

    def onMenuAnalyze(self, enableResynthesis = True , exportSpecification = False):
        "simplified version of that in specEditor.py"

        # instantiate if necessary (instantiate everytime since we are using showModal here
        Editor = wx.PySimpleApp(0)
        self.analysisDialog = livenessEditor.AnalysisResultsDialog(self,None)  #parent as resynthesis now

        # Clear dialog to make way for new info
        self.analysisDialog.text_ctrl_summary.Clear()

        # Populate tree based on .spec file
        self.analysisDialog.label_traceback.Show()
        self.analysisDialog.tree_ctrl_traceback.Show()
        
        if not enableResynthesis:
            self.analysisDialog.button_refine.Disable()
        
        if not exportSpecification:
            self.analysisDialog.button_2.Disable() 
                   
        self.analyzeCores()
        self.analysisDialog.populateTreeStructured(self.proj.specText.split('\n'),self.compiler.LTL2SpecLineNumber, self.tracebackTree, [], self.spec,self.to_highlight,self.spec["EnvTrans"].replace('\t','\n')) # [] was self.EnvTransRemoved
        
        logging.debug('toHighlight in analyze cores'+ str(self.to_highlight))
        self.analysisDialog.ShowModal()
        
    def onMenuResynthesize(self, envLiveness, checker=None):
        """
        Resynthesize the specification with a new livenessEditor
        envLiveness: new env liveness from the user
        """
        if checker is None:
            checker = self.LTLViolationCheck

        ############ COPIED FROM SPECCOMPILER LINE 285 ###########
        if self.proj.compile_options["decompose"]:
            # substitute the regions name in specs
            for m in re.finditer(r'near (?P<rA>\w+)', envLiveness):
                envLiveness=re.sub(r'near (?P<rA>\w+)', "("+' or '.join(["s."+r for r in self.proj.regionMapping['near$'+m.group('rA')+'$'+str(50)]])+")", envLiveness)
            for m in re.finditer(r'within (?P<dist>\d+) (from|of) (?P<rA>\w+)', envLiveness):
                envLiveness=re.sub(r'within ' + m.group('dist')+' (from|of) '+ m.group('rA'), "("+' or '.join(["s."+r for r in self.proj.regionMapping['near$'+m.group('rA')+'$'+m.group('dist')]])+")", envLiveness)
            for m in re.finditer(r'between (?P<rA>\w+) and (?P<rB>\w+)', envLiveness):
                envLiveness=re.sub(r'between ' + m.group('rA')+' and '+ m.group('rB'),"("+' or '.join(["s."+r for r in self.proj.regionMapping['between$'+m.group('rA')+'$and$'+m.group('rB')+"$"]])+")", envLiveness)

            # substitute decomposed region
            for r in self.proj.regionMapping.keys():
                envLiveness = re.sub('\\b' + r + '\\b', "("+' | '.join(["s."+x for x in self.proj.regionMapping[r]])+")", envLiveness)

        else:
            for r in self.proj.regionMapping.keys():
                if not (r.lower() == "boundary"):
                    envLiveness = re.sub('\\b' + r + '\\b', "s."+r, envLiveness)

        regionList = ["s."+x.name for x in self.proj.rfi.regions]
        regionListCompleted = [x.name+"_rc" for x in self.proj.rfi.regions]
        ##################################################

        try:
            spec, traceback, failed, LTL2SpecLineNumber, internal_props = parseEnglishToLTL.writeSpec(envLiveness, \
                self.compiler.sensorList, self.compiler.regionList, self.compiler.actuatorList,\
                self.compiler.customsList, fastslow=self.proj.compile_options['fastslow'])

            if self.proj.compile_options["use_region_bit_encoding"]:
                # Define the number of bits needed to encode the regions
                numBits = int(math.ceil(math.log(len(regionList),2)))

                # creating the region bit encoding
                bitEncode = parseEnglishToLTL.bitEncoding(len(regionList),numBits)

                # switch to bit encodings for regions
                spec["EnvGoals"] = parseEnglishToLTL.replaceRegionName(spec["EnvGoals"], bitEncode, regionList)
                spec["EnvGoals"] = parseEnglishToLTL.replaceRegionName(spec["EnvGoals"], bitEncode, regionListCompleted)

        except:
            logging.debug('envLiveness:' + str(envLiveness))
            self.analysisDialog.appendLog("\nERROR: Aborting compilation due to syntax error. \nPlease enter environment liveness with correct grammar\n", "RED")
            return
        else:
            if failed:
                self.analysisDialog.appendLog("\nERROR: Aborting compilation due to syntax error. \nPlease enter environment liveness with correct grammar\n", "RED")
                return

        # create a copy of the current spec just for resynthesis
        currentSpec = self.spec.copy()
        
        # remove []<>(TRUE) if we find it
        if "TRUE" in self.spec["EnvGoals"]: 
            currentSpec["EnvGoals"] =  spec["EnvGoals"]  #envLiveness
        else:
            currentSpec["EnvGoals"] += ' &\n' + spec["EnvGoals"] #envLiveness
            
        for x in range(len(checker.env_safety_assumptions_stage)):
            checker.modify_stage  = x+1
            currentSpec["EnvTrans"] = checker.modify_LTL_file("")
            self.recreateLTLfile(self.proj,currentSpec)
            slugsEnvSafetyCNF, normalEnvSafetyCNF = self.exportSpecification(appendLog = False)
            
            if self.analyzeCores(appendLog = False):
                break

        self.postEvent("INFO","Reset stage to " + str(checker.modify_stage))
        
        if self.analyzeCores():
            self.spec = currentSpec  # realizable, replace the new spec to be the self.spec
            
            # save the user added liveness for display in analysis dialog
            self.userAddedEnvLivenessEnglish.append(envLiveness)
            self.userAddedEnvLivenessLTL.append(spec["EnvGoals"].replace("\t",'').replace("\n",'').replace(" ",""))
            
            #reprint the tree in the analysis dialog
            self.analysisDialog.populateTreeStructured(self.proj.specText.split('\n'),self.compiler.LTL2SpecLineNumber, self.tracebackTree, [], self.spec,self.to_highlight,self.spec["EnvTrans"].replace('\t','\n')) # [] was self.EnvTransRemoved
         
        else:
            # return the ltl file back to normal as the newly added liveness is still unrealizable
            self.recreateLTLfile(self.proj,self.spec)  
    
    def analyzeCores(self,appendLog = True, generatedSpec = False):
        """
        EXCERPT from Vasu's code in specEditor.py
        """
        #TODO: not sure why realizable is false from GROneDebug even when it is realizable
        (realizable, self.unsat, nonTrivial, self.to_highlight, outputFromAnalyze) = self.compiler._analyze(generatedSpec)
        realizable, realizableFS, outputFromSythesize = self.compiler._synthesize()          
        
        if realizable:
            output = outputFromSythesize
        else:
            output = outputFromAnalyze
            
        # Remove lines about garbage collection from the output and remove extraenous lines
        output_lines = [line for line in output.split('\n') if line.strip() and
                        "Garbage collection" not in line and
                        "Resizing node table" not in line]

        if appendLog:              
            if realizable:
                # Strip trailing \n from output so it doesn't scroll past it
                self.analysisDialog.appendLog("\n"+'\n'.join(output_lines), "BLACK")
            else:
                self.analysisDialog.appendLog(output.rstrip(), "RED")
            self.analysisDialog.appendLog('\n')
        
        return realizable

            
    def exportSpecification(self, appendLog = True):
        """
        export the generated spec
        """
        slugsEnvSafetyCNF = self.compiler._synthesize()[2]
        normalEnvSafetyCNF = LTLParser.LTLcheck.parseSlugsEnvTransToNormalEnvTrans(slugsEnvSafetyCNF,self.proj.enabled_sensors)         
            
        self.originalLTLSpec["EnvGoals"] = self.spec["EnvGoals"]
        self.recreateLTLfile(self.proj, self.originalLTLSpec, export = True)
        if appendLog:
            self.analysisDialog.appendLog("\nThe generated specification with simplified environment safety assumptions is exported to " + self.proj.getFilenamePrefix()+"Generated.ltl","BLACK")
            
            self.analyzeCores(appendLog = True, generatedSpec = True)
            
        return slugsEnvSafetyCNF, normalEnvSafetyCNF
        
    #########################################################################

    # ************** patching **************** #
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
            logging.warning('not in a valid mode - ' + str(self.proj.compile_options["multi_robot_mode"]))

        # first verify that we won't be skipping th check
        if list(set(otherEnvPropDict.keys()+[self.dPatchingExecutor.robotName])) != robotNameList:
            logging.debug("we are checking propCombination")
            for propCombination in itertools.product(*[v for k,v in otherEnvPropDict.iteritems() if k not in robotNameList]):
                #logging.debug("propCombination:" + str(propCombination))
                # assignments to sensor_strategy for each combination
                for propDict in propCombination:
                    for propKey, propValue in propDict.iteritems():
                        # check if key exist in sensor strategy?
                        if propKey in deepcopy_sensor_state.getInputs(expand_domains=True).keys():
                            deepcopy_sensor_state.setPropValue(propKey, propValue)

                    # the current state stays the same but checks with differnt next possible states
                    env_assumption_hold = checker.checkViolation(deepcopy_current_state, deepcopy_sensor_state)
                    if not env_assumption_hold:
                        logging.debug("POSSIBLE STATES - asssumptions violated!")
                        logging.debug("current_state:" + str([k for k, v in deepcopy_current_state.getAll(expand_domains=True).iteritems() if v]))
                        logging.debug("sensor_state" + str([k for k, v in deepcopy_sensor_state.getInputs(expand_domains=True).iteritems() if v]))
                        logging.debug("checker.violated_specStr:" + str(checker.violated_specStr))
                        return False
        else:
            logging.debug("we have all the robots. doing only once")
            env_assumption_hold = checker.checkViolation(deepcopy_current_state, deepcopy_sensor_state)
            if not env_assumption_hold:
                logging.debug("POSSIBLE STATES - asssumptions violated!")
                logging.debug("current_state:" + str([k for k, v in deepcopy_current_state.getAll(expand_domains=True).iteritems() if v]))
                logging.debug("sensor_state" + str([k for k, v in deepcopy_sensor_state.getInputs(expand_domains=True).iteritems() if v]))
                logging.debug("checker.violated_specStr:" + str(checker.violated_specStr))
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
                logging.warning("SLUGS does not output bddLivenesses now and this will fail.")
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
        logging.debug("Using sensor state. Should be the violation one" + str(self.sensor_strategy.getInputs()))
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

        #logging.debug("sensorList:" + str(sensorList))
        # convert formula from slugs to our format
        sysGoalsLTLList = LTLParser.translateFromSlugsLTLFormatToLTLFormat.parseSLUGSCNFtoLTLList(slugsStr,sensorList)
        #logging.debug(sysGoalsLTLList)

        # replace with normal region bits (like actual regions)
        sysGoalsLTL = self.replaceIndividualSbitsToGroupSbits(sysGoalsLTLList)
        #logging.debug(sysGoalsLTL)

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

        #logging.debug("sensorList:" + str(sensorList))
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
    # **************************************** #

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
                #logging.debug("specNewStr:" + str(specNewStr))
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
        logging.debug("Using sensor state. Should be the violation one" + str(self.sensor_strategy.getInputs()))
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
        logging.debug("we should have finished setting up things for ourselves")

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
        logging.debug("list(set(self.violated_spec_list + self.possible_states_violated_spec_list)):" + str(list(set(self.violated_spec_list + self.possible_states_violated_spec_list))))
        logging.debug("robotsInConflict:" + str(robotsInConflict))
        if robotsInConflict: # list not empty. Some robots is in conflict with us
            #self.dPatchingExecutor.coordinationRequestSent = robotsInConflict
            self.dPatchingExecutor.setCoordinationRequestSent(robotsInConflict)
            self.postEvent("D-PATCH","We will now ask for a centralized strategy be executed.")
            self.runCentralizedStrategy = True
        elif received_request:
            logging.info("We are asked to join patching")
            self.runCentralizedStrategy = True

        else:
            logging.warning("we need to trigger env characterization instead. This is not checked yet!")
            self.postEvent("INFO","Violation is only about our propositions. Carrying out environment characterization")
            logging.debug("list(set(self.violated_spec_list + self.possible_states_violated_spec_list)):" + str(list(set(self.violated_spec_list + self.possible_states_violated_spec_list))))
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
        logging.warning('self.hsub.getSensorValue(enabled_sensors):' + str(self.hsub.getSensorValue(enabled_sensors)))
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

        logging.debug('globalEnvTrans:' + str(globalEnvTrans))
        # globalEnvTransList = []
        # for robot in self.dPatchingExecutor.coordinatingRobots:
        #     globalEnvTransList.append(self.dPatchingExecutor.filterAndExcludeSpecOfCoordinatingRobots(self.spec['EnvTrans'][robot], robot))
        # globalEnvTrans = " &\n".join(filter(None, ))

        self.globalEnvTransCheck = LTLParser.LTLcheck.LTL_Check(None, {}, {'EnvTrans': globalEnvTrans}, 'EnvTrans')
        self.globalEnvTransCheck.ltl_treeEnvTrans = LTLParser.LTLFormula.parseLTL(globalEnvTrans)
        logging.debug('finished setupGlobalEnvTransCheck')
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
        logging.debug("violatedList:" + str(violatedList))
        violatedListFiltered = LTLParser.LTLcheck.filterOneRobotViolatedSpec(violatedList, self.dPatchingExecutor.robotInRange + [self.dPatchingExecutor.robotName])
        logging.debug("violatedListFiltered:" + str(violatedListFiltered))

        specNewStr = LTLParser.LTLcheck.excludeSpecFromFormula(specStr, violatedListFiltered)
        #specNewStr = "&\n".join(filter(None,violatedListFiltered))
        logging.debug("specNewStr:" + specNewStr)
        return specNewStr

    def synthesizeGlobalStrategyAndWaitForOthersToResume(self):
        """
        This function asks the dPatchingExecutor to synthesize global strategy.
        After that it waits till all coorindating robots are ready to restart.
        """
        # then wait till all surrounding robots are ready as well
        while not self.dPatchingExecutor.prepareForCentralizedExecution():
            logging.warning('we are still waiting parts from the other robots')

            self.dPatchingExecutor.runIterationNotCentralExecution()
            logging.debug("self.dPatchingExecutor.coordinatingRobots:" + str(self.dPatchingExecutor.coordinatingRobots))
            time.sleep(0.2)

        # now wait till the other robot has synthesized an automaton
        # TODO: what if it's unrealizable? We should know that as well.
        while not self.dPatchingExecutor.checkIfOtherRobotsAreReadyToExecuteCentralizedStrategy():
            logging.warning('we are still waiting completion of synthesis from the other robots')

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
            #    logging.debug("Robot Under consideration:" + str(robot))
            #    self.dPatchingExecutor.spec['EnvTrans'][robot] = self.filterAndExcludeSpec(list(set(self.violated_spec_list + self.possible_states_violated_spec_list)), self.dPatchingExecutor.spec['EnvTrans'][robot])
            #    self.setupGlobalEnvTransCheck() # update EnvTrans globel by setting up a new object
            #    logging.debug("-------------------------------------------")

            # add violations of local spec to LTL violated_str list
            for specStr in list(set(self.violated_spec_list + self.possible_states_violated_spec_list)):
                # replace e.g. alice_r1_rc to alice_r1 as the meaning changes in the central strategy
                for region in self.dPatchingExecutor.regionList:
                    for otherRobot in list(set(self.dPatchingExecutor.coordinationRequestSent) | set([robot for robot in self.dPatchingExecutor.coordinationRequest.keys()])):
                        specStr = re.sub('(?<=[! &|(\t\n])e.'+otherRobot+'_'+region+'_rc'+'(?=[ &|)\t\n])', 'e.'+otherRobot+'_'+region, specStr)
                self.LTLViolationCheck.violated_specStr.append(specStr)
                self.LTLViolationCheckPossibleStates.violated_specStr.append(specStr)

            logging.debug("self.LTLViolationCheck.violated_specStr (also LTLViolationCheckPossibleStates):" + str(self.LTLViolationCheck.violated_specStr))

            #################################################
            #### CHECK IF NEW ROBOTS ARE INVOLVED #########
            #################################################
            # first checks if we are already coorindating with the robots in conflict
            if set(robotsInConflict).issubset(self.dPatchingExecutor.coordinatingRobots):
                # reset coordination status
                self.dPatchingExecutor.resetRobotStatusOnCentralizedStrategyExecution()

                # all the robots in conflicts are already coorindating. Just remove violated line and resynthesize.
                logging.warning("in conflict robots are the same. just remove envTrans and resynthesize")
                self.postEvent("RESOLVED","Current coordinating robots only. Removing violated environment assumptions and resynthesize.")

            else:
                # some new robots are joining, we should ask them to join centralized strategy.
                logging.warning("invite other robots to join the centralized execution")

                # send spec to request received and also violated robots
                csockNewRobotsToCoordinate = [self.dPatchingExecutor.clients[robot] for robot in robotsInConflict if not robot in self.dPatchingExecutor.old_coordinatingRobots]

                #only send local spec here.(Assuming the other robot coordinating find out the same problem and send its own)
                self.prepareForCentralizedStrategySnippetToOtherRobots(csockNewRobotsToCoordinate)

                #update coordination sent
                #self.dPatchingExecutor.coordinationRequestSent = robotsInConflict
                self.dPatchingExecutor.setCoordinationRequestSent(robotsInConflict)

        elif received_request:
            logging.info("We are asked to coordinate with more robots!")
        else:
            self.postEvent("INFO","Violation is only about our propositions. Carrying out environment characterization")
            self.addStatetoEnvSafety(self.dPatchingExecutor.sensor_state, checker=self.globalEnvTransCheck)
            logging.warning("we need to trigger env characterization instead. This is not checked!")

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
