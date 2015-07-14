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
        self.otherRobotStatus = None #track if we are prioritized. False: env Characterization enabled. True: env Char disabled. None: no negotiation 
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
    def synthesizeWithExchangedSpec(self, level = False):
        """
        spec negotiation in execute.py
        """

        # fist append new environment /and system init state
        self._setSpecificationInitialConditionsToCurrentInDNF(self.proj,False, self.sensor_strategy)

        # reset env characterization
        self.spec['EnvTrans'] = self.oriEnvTrans
        self.LTLViolationCheck.replaceLTLTree(self.oriEnvTrans)
        self.spec['EnvTrans'] = '[](('+self.spec['EnvTrans'].replace('[]','') +'))\n'
        self.LTLViolationCheck.env_safety_assumptions_stage = {"1": self.spec['EnvTrans'][:-3] , "3": self.spec['EnvTrans'][:-3] , "2": self.spec['EnvTrans'][:-3] } # we are striping the last ))

        # obtain SysGoals, EnvTrans of the other robot 
        # may not have anything the other robot have not sent info. (dealt with inside requestSpec) -- may move the check here.
        otherRobotSysGoals = self.robClient.requestSpec('SysGoals')
        otherRobotEnvTrans = self.robClient.requestSpec('EnvTrans').replace('[]','\n\t\t\t[]')
        self.receivedSpec = True

        # see if we can take the other robot's actions into account. 
        # first we safe a copy of our SysTrans and EnvGoals before modification
        #oldSpecSysTrans = self.spec['SysTrans'].replace('\t',"").replace(' ','').replace('\n','')
        #oldSpecEnvGoals = self.spec['EnvGoals'].replace('\t',"").replace(' ','').replace('\n','')
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
        realizable, realizableFS, output  = self.compiler._synthesize()
       
        return realizable, oldSpecSysTrans, oldSpecEnvGoals 
        
    def appendSpecFromEnvRobots(self):
        """
        asked for spec from negotiation monitor and append to our spec.
        """

        # make resynthesis with recovery
        self.recovery = True
        self.proj.compile_options['recovery'] = True
        self.compiler.proj.compile_options['recovery'] = True

        # modify envTrans to remove all characterization. Restart Characterization
        self.spec['EnvTrans'] = self.oriEnvTrans
        self.LTLViolationCheck.replaceLTLTree(self.oriEnvTrans)
        #self.spec['EnvTrans'] = '[](('+self.spec['EnvTrans'].replace("\t","").replace("\n","").replace(" ","").replace('[]','')[:-1] +'))&\n'
        self.spec['EnvTrans'] = '[](('+self.spec['EnvTrans'].replace('[]','') +'))\n'
        self.LTLViolationCheck.env_safety_assumptions_stage = {"1": self.spec['EnvTrans'][:-3] , "3": self.spec['EnvTrans'][:-3] , "2": self.spec['EnvTrans'][:-3] }
        self.postEvent('NEGO','-- NEGOTIATION STARTED --')                
        self.postEvent('NEGO','Ask the other robot to include our actions in its controller.')
        # send SysGoals, EnvTrans and EnvGoals
        self.robClient.sendSpec('SysGoals',self.spec['SysGoals'], self.proj.compile_options["fastslow"], self.proj.compile_options["include_heading"])
        if self.proj.compile_options["fastslow"]:
            self.robClient.sendSpec('EnvTrans', LTLParser.LTLcheck.removeLTLwithoutKeyFromEnvTrans(self.oriEnvTrans, self.proj.otherRobot[0]), self.proj.compile_options["fastslow"], self.proj.compile_options["include_heading"])
        else:
            self.robClient.sendSpec('EnvTrans',self.spec['EnvTrans'])
        #self.robClient.sendSpec('EnvGoals',self.spec['EnvGoals']) 
        self.sentSpec = True

        self.robClient.setNegotiationStatus("'" + self.proj.otherRobot[0] + "'")
        while self.robClient.checkNegotiationStatus() == self.proj.otherRobot[0]:
            # wait for the actions of the other robot
            time.sleep(2)
        
        if self.robClient.checkNegotiationStatus() == True:
            # we need to use our original spec
            self.spec = copy.deepcopy(self.originalSpec)
            self.spec['EnvTrans'] = self.oriEnvTrans
            #convert to the original specification
            self._setSpecificationInitialConditionsToCurrentInDNF(self.proj,False, self.sensor_strategy)
            self.recreateLTLfile(self.proj, spec = self.spec)
            realizable, realizableFS, output  = self.compiler._synthesize()
            logging.debug("Original spec with new env/sys init realizable?" + str(realizable))
            self.otherRobotStatus = True # env characterization disabled
            self.postEvent('NEGO','The other robot has incorporated our actions. We will use the original spec')
            self.postEvent('NEGO','-- NEGOTIATION ENDED --')
            self.postEvent('RESOLVED','')
            
        elif self.robClient.checkNegotiationStatus() == self.robClient.robotName and not self.receivedSpec:
            self.postEvent('NEGO','The other robot cannot incorporate our actions. We will try incorporating its actions instead.')
            # try to synthesize controller with spec from the other robot instead
            ###### ONE STEP ####
            realizable, oldSpecSysTrans_2, oldSpecEnvGoals_2 = self.synthesizeWithExchangedSpec(False)
            self.postEvent("NEGO",'Adding system guarantees with environment goals.')
            
            if realizable:
                self.postEvent('NEGO','We can continue with the exchanged information.')
                self.postEvent('NEGO','-- NEGOTIATION ENDED --')
                self.postEvent('RESOLVED','')
                self.robClient.setNegotiationStatus(True)
                self.otherRobotStatus = False # env characterization enabled
            else:
                self.postEvent('NEGO','Negotiation failed')
                self.postEvent('NEGO','-- NEGOTIATION ENDED --')
                self.robClient.setNegotiationStatus(False)
                self.otherRobotStatus = False # env characterization enabled

        else: #We have received spec before and it's unrealizable
            realizable = False
            self.postEvent('NEGO','Negotiation failed')
            self.postEvent('NEGO','-- NEGOTIATION ENDED --')
            self.robClient.setNegotiationStatus(False)
            self.otherRobotStatus = False # env characterization enabled

        # reset violtion timestamp
        self.violationTimeStamp = 0
        self.robClient.setViolationTimeStamp(self.violationTimeStamp)
        logging.debug('Resetting violation timeStamp')
        time.sleep(1)
        """
        # --------------------------------------------- #
        # check if the other robot is realizable. 
        otherRobotsStrategyStatus = self.robClient.requestStrategyStatus()
        # remove current robot status from the dict
        otherRobotsStrategyStatus.pop(self.robClient.robotName, None)
        logging.debug('otherRobotsStrategyStatus: ' + str(otherRobotsStrategyStatus))
        
        # force the spec to use the original spec
        if True in otherRobotsStrategyStatus.values():
            realizable = False
        
        # notify negotiation monitor our controller statues
        self.robClient.updateStrategyStatus(realizable)  
        # --------------------------------------------- #
        
        logging.debug('realizable status sent')
        if not realizable:
            self.postEvent("VIOLATION","Controller with exchanged information is unrealizable. Removing exchanged info from the spec")  
            
            # remove spec from other robots
            self.spec['SysTrans'] = oldSpecSysTrans
            self.spec['EnvGoals'] = oldSpecEnvGoals
            self.recreateLTLfile(self.proj)
            realizable, realizableFS, output  = self.compiler._synthesize()
            
        else:
            self.postEvent("INFO",'Specification is realizable with the exchanged info.')    
        
        time.sleep(2)
        # check if the other robot is realizable. 
        otherRobotsStrategyStatus = self.robClient.requestStrategyStatus()
        logging.debug(otherRobotsStrategyStatus)
        
        # remove current robot status from the dict
        otherRobotsStrategyStatus.pop(self.robClient.robotName, None)
        logging.debug('otherRobotsStrategyStatus: ' + str(otherRobotsStrategyStatus))
        
        if True in otherRobotsStrategyStatus.values() and not realizable:
            # Yes.. it's great.. we can use our old spec. Maybe need liveness.
            self.postEvent('RESOLVED','We can continue with the old spec as the other robot will yield for us.')
            self.otherRobotStatus = True
       
        #TODO: making one with exchanged spec and the other with the original spec
        elif True in otherRobotsStrategyStatus.values() and realizable:
            self.postEvent('RESOLVED','Determing one to be the priority.')
            #TODO: currently assuming two robots. This has to be changed later. 
            if self.robClient.robotName < otherRobotsStrategyStatus.keys()[0]:
                # We are the priority. Stay with the original spec
                self.otherRobotStatus = True
                self.postEvent('INFO','I am prioritized. Using original Spec')
                
                # remove spec from other robots and resynthesize
                self.spec['SysTrans'] = oldSpecSysTrans
                self.spec['EnvGoals'] = oldSpecEnvGoals
                self.recreateLTLfile(self.proj)
                realizable, realizableFS, output  = self.compiler._synthesize()
            else:
                # We will yeild to the other robot.
                self.otherRobotStatus = False
                self.postEvent('INFO','I am NOT prioritized. Yield to the other robot.')
       
        elif realizable:
            self.postEvent('RESOLVED','We will yield to the other robot.')
            self.otherRobotStatus = False 
            
        else:
            # TODO: NO.. what should we do
            self.postEvent('INFO','What should we do.. Both of us are unrealizable... ')
            self.otherRobotStatus = False
        """
        return realizable             

    def receiveRequestFromEnvRobot(self):
        """
        check if a request for negotiation is received.
        Returns true if negotiation occurs and false otherwise
        """
        # resynthesis request from the other robot
        self.negotiationStatus = self.robClient.checkNegotiationStatus()
        if (not self.exchangedSpec or not self.receivedSpec) and self.negotiationStatus == self.robClient.robotName:

            # make resynthesis with recovery
            self.recovery = True
            self.proj.compile_options['recovery'] = True
            self.compiler.proj.compile_options['recovery'] = True

            self.postEvent('NEGO','-- NEGOTIATION STARTED --')
            # synthesize a new controller to incorporate the actions of the other robot.
            ###### ONE STEP ######
            self.realizable, oldSpecSysTrans_2, oldSpecEnvGoals_2 = self.synthesizeWithExchangedSpec(False)
            self.postEvent("NEGO",'Adding system guarantees with environment goals.')

            if self.realizable:
                self.robClient.setNegotiationStatus(True)
                self.otherRobotStatus = False
                self.postEvent('NEGO','Using exchanged specification.')
                self.postEvent('RESOLVED','')

                # reset violtion timestamp
                self.violationTimeStamp = 0
                self.robClient.setViolationTimeStamp(self.violationTimeStamp)
                logging.debug('Resetting violation timeStamp')
                time.sleep(1)

                self.exchangedSpec = True
                # reinitialize automaton
                spec_file = self.proj.getFilenamePrefix() + ".spec"
                aut_file = self.proj.getFilenamePrefix() + ".aut"
                self.initialize(spec_file, aut_file, firstRun=False)

            elif not self.sentSpec:
                self.postEvent('NEGO','Unrealizable with exchanged info. Asking the other robot to incorporate our actions instead.')

                # send our spec to the other robot
                self.robClient.sendSpec('SysGoals',self.spec['SysGoals'], self.proj.compile_options["fastslow"], self.proj.compile_options["include_heading"])
                if self.proj.compile_options["fastslow"]:
                    self.robClient.sendSpec('EnvTrans', LTLParser.LTLcheck.removeLTLwithoutKeyFromEnvTrans(self.oriEnvTrans, self.proj.otherRobot[0]), self.proj.compile_options["fastslow"], self.proj.compile_options["include_heading"])
                else:
                    self.robClient.sendSpec('EnvTrans',self.spec['EnvTrans'])

                #self.robClient.sendSpec('EnvGoals',self.spec['EnvGoals'])
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
                    self.realizable, realizableFS, output  = self.compiler._synthesize()
                    logging.debug("Original spec with new env/sys init realizable?" + str(self.realizable))
                    self.otherRobotStatus = True # env characterization disabled
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
    def addStatetoEnvSafety(self,  sensor_state, firstRun = False):
        """
        append the current detected state to our safety
        """
        
        if (self.otherRobotStatus is None) or (not self.otherRobotStatus):
            # Add the current state in init state of the LTL spec
            self.postEvent("VIOLATION","Adding the current state to our initial conditions")
            self._setSpecificationInitialConditionsToCurrentInDNF(self.proj,firstRun, sensor_state)
            
            if not firstRun:
                self.spec['EnvTrans'] = self.LTLViolationCheck.modify_LTL_file("")
            
            self.recreateLTLfile(self.proj)
            realizable, realizableFS, output = self.compiler._synthesize()  # TRUE for realizable, FALSE for unrealizable
             
            if not firstRun:
                self.postEvent("VIOLATION",self.simGUILearningDialog[self.LTLViolationCheck.modify_stage-1] + " and the specification is " + ("realizable." if realizable else "unrealizable."))
                if not realizable:
                    while self.LTLViolationCheck.modify_stage < 2 and not realizable:   # < 3 and not realizable:        # CHANGED TO 2 FOR PAPER

                        self.LTLViolationCheck.modify_stage += 1 
                        self.spec['EnvTrans'] = self.LTLViolationCheck.modify_LTL_file("")  
                        self.recreateLTLfile(self.proj)
                        realizable, realizableFS, output  = self.compiler._synthesize()  # TRUE for realizable, FALSE for unrealizable
                        
                        self.postEvent("VIOLATION",self.simGUILearningDialog[self.LTLViolationCheck.modify_stage-1] + " and the specification is " + ("realizable." if realizable else "unrealizable."))

                    if self.proj.compile_options['neighbour_robot']:
                        if not realizable:
                            #realizable = False
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

            self.realizable = realizable

        else:
            logging.debug('Waiting for the other robot to yield our way.')
            #logging.debug(self.lastSensorState.getInputs())
            #logging.debug(sensor_state.getInputs())

            # only append new init state if there's no request of negotiation
            if not self.receiveRequestFromEnvRobot():
                if self.lastSensorState is None  or self.lastSensorState.getInputs(expand_domains=True) != sensor_state.getInputs(expand_domains=True) or not self.realizable:
                    # update spec with current state of the other robot
                    self._setSpecificationInitialConditionsToCurrentInDNF(self.proj,firstRun, sensor_state)
                    self.recreateLTLfile(self.proj)
                    self.realizable, realizableFS, output = self.compiler._synthesize()  # TRUE for realizable, FALSE for unrealizable
                    self.postEvent('INFO','Recreating automaton based on the new env init state.')

        self.lastSensorState = sensor_state   
        """      
        if not realizable:
            self.postEvent("VIOLATION","Please enter some environment liveness to make the specification realizable.")
            # Use Vasu's analysis tool 
            self.onMenuAnalyze()  # in resynthesis.py
            realizable = self.compiler._synthesize()[0]  # TRUE for realizable, FALSE for unrealizable                  
            
            if not realizable:
                self.postEvent("VIOLATION", "Specification is still unrealizable after adding env liveness. Now we will exit the execution")
                sys.exit()
            else:                   
                self.LTLViolationCheck.modify_stage  = 1 # reset the stage to 1
        """

             
        # reload aut file if the new ltl is realizable                  
        if self.realizable or self.otherRobotStatus:
            if not self.otherRobotStatus:
                self.postEvent("RESOLVED", "The specification violation is resolved.")
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
        if firstRun:
            tempY = []
            # figure out our current region
            init_region = self._getCurrentRegionFromPose()
            numBits = int(math.ceil(math.log(len(self.proj.rfi.regions),2)))          
            reg_idx_bin = numpy.binary_repr(init_region, width=numBits)
            
            for x in range(numBits):
                if init_region & 2**x:
                    tempY.append("s.bit" + str(x))
                else:
                    tempY.append("!s.bit" + str(x))
                             
            # Figure out our initially true outputs
            init_outputs = []
            for prop in self.proj.currentConfig.initial_truths:
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
                for x in range(max(1, int(math.ceil(math.log(len([1,2,3,4,5]), 2))))):
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

        # adding also other initial mutual exclusions
        if self.proj.compile_options['neighbour_robot']:
            self.spec["EnvInit"] += " &\n " + self.spec['InitEnvRegionSanityCheck']

        self.spec["SysInit"]  = "(" + current_sys_init_state.replace("\t", "").replace("\n", "").replace(" ", "") + ")"

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
        LTLspec_env = ' &\n'.join(filter(None,[spec["EnvInit"], spec["EnvTrans"], spec["EnvGoals"]]))
        LTLspec_sys = ' &\n'.join(filter(None,[spec["SysInit"], spec["SysTrans"], spec["SysGoals"], spec['InitRegionSanityCheck'], spec['Topo']]))

        if proj.compile_options["fastslow"]:
            LTLspec_env += "\n&"+spec['SysImplyEnv']

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
        
    def onMenuResynthesize(self, envLiveness):
        """
        Resynthesize the specification with a new livenessEditor
        envLiveness: new env liveness from the user
        """

        try:
            spec, traceback, failed, LTL2SpecLineNumber, internal_props = parseEnglishToLTL.writeSpec(envLiveness, self.compiler.sensorList, self.compiler.regionList, self.compiler.robotPropList)
        except:
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
            
        for x in range(len(self.LTLViolationCheck.env_safety_assumptions_stage)):
            self.LTLViolationCheck.modify_stage  = x+1 
            currentSpec["EnvTrans"] = self.LTLViolationCheck.modify_LTL_file("")
            self.recreateLTLfile(self.proj,currentSpec)
            slugsEnvSafetyCNF, normalEnvSafetyCNF = self.exportSpecification(appendLog = False)
            
            if self.analyzeCores(appendLog = False):
                break

        self.postEvent("INFO","Reset stage to " + str(self.LTLViolationCheck.modify_stage))
        
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
