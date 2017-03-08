#!/usr/bin/env python

""" =================================================
    execute.py - Top-level hybrid controller executor
    =================================================

    This module executes a hybrid controller for a robot in a simulated or real environment.

    :Usage: ``execute.py [-hn] [-p listen_port] [-a automaton_file] [-s spec_file]``

    * The controlling automaton is imported from the specified ``automaton_file``.

    * The supporting handler modules (e.g. sensor, actuator, motion control, simulation environment initialization, etc)
      are loaded according to the settings in the config file specified as current in the ``spec_file``.

    * If no port to listen on is specified, an open one will be chosen randomly.
    * Unless otherwise specified with the ``-n`` or ``--no_gui`` option, a status/control window
      will also be opened for informational purposes.
"""

import sys, os, getopt, textwrap
import threading, subprocess, time
import pdb, traceback

# Climb the tree to find out where we are
p = os.path.abspath(__file__)
t = ""
while t != "src":
    (p, t) = os.path.split(p)
    if p == "":
        print "I have no idea where I am; this is ridiculous"
        sys.exit(1)

sys.path.append(os.path.join(p,"src","lib"))

import fsa, project
import handlerSubsystem
import strategy
from copy import deepcopy
from SimpleXMLRPCServer import SimpleXMLRPCServer, SimpleXMLRPCRequestHandler
import xmlrpclib
import socket
import random
import math
import traceback
from resynthesis import ExecutorResynthesisExtensions
from executeStrategy import ExecutorStrategyExtensions
from executeModes import ExecutorModesExtensions
from executePatching import ExecutorPatchingExtensions
from executeDecentralizedPatching import ExecutorDecentralizedPatchingExtensions
import globalConfig

# logger for ltlmop
import logging
ltlmop_logger = logging.getLogger('ltlmop_logger')

###### ENV VIOLATION CHECK ######
import copy
import specCompiler

import LTLParser.LTLcheck
import logging
import LTLParser.LTLFormula 
#################################

# -----------------------------------------#
# -------- two_robot_negotiation ----------#
# -----------------------------------------#
import negotiationMonitor.robotClient
# -----------------------------------------#

# ********* patching ************** #
import itertools #for iterating props combination
# ********************************* #

# %%%%%%%%% d-patching %%%%%%%%%%% #
import centralCoordinator.decentralizedPatchingExecutor
# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% #

import handlers.handlerTemplates as ht


####################
# HELPER FUNCTIONS #
####################

def usage(script_name):
    """ Print command-line usage information. """

    print textwrap.dedent("""\
                              Usage: %s [-hn] [-p listen_port] [-a automaton_file] [-s spec_file]

                              -h, --help:
                                  Display this message
                              -n, --no-gui:
                                  Do not show status/control window
                              -p PORT, --xmlrpc-listen-port PORT:
                                  Listen on PORT for XML-RPC calls
                              -a FILE, --aut-file FILE:
                                  Load automaton from FILE
                              -s FILE, --spec-file FILE:
                                  Load experiment configuration from FILE """ % script_name)

class LTLMoPExecutor(ExecutorStrategyExtensions, ExecutorResynthesisExtensions, ExecutorModesExtensions, ExecutorPatchingExtensions, ExecutorDecentralizedPatchingExtensions, object):
    """
    This is the main execution object, which combines the synthesized discrete automaton
    with a set of handlers (as specified in a .config file) to create and run a hybrid controller
    """

    def __init__(self):
        """
        Create a new execution context object
        """
        super(LTLMoPExecutor, self).__init__()

        self.proj = project.Project() # this is the project that we are currently using to execute
        self.strategy = None

        # Choose a timer func with maximum accuracy for given platform
        if sys.platform in ['win32', 'cygwin']:
            self.timer_func = time.clock
        else:
            self.timer_func = time.time

        self.externalEventTarget = None
        self.externalEventTargetRegistered = threading.Event()
        self.postEventLock = threading.Lock()
        self.runStrategy = threading.Event()  # Start out paused
        self.runRuntimeMonitoring = threading.Event() # track if runtime monitoring should be run
        self.alive = threading.Event()
        self.alive.set()

        self.current_outputs = {}     # keep track on current outputs values (for actuations)
        ########## ENV Assumption Learning ######
        self.compiler = None                   
        self.LTLViolationCheck = None
        self.LTLViolationCheckPossibleStates = None
        self.analysisDialog = None
        self.to_highlight = None
        self.tracebackTree = None               # tells you init, trans and sys line no 
        self.path_LTLfile = None                    # path of the .ltl file
        self.LTL2SpecLineNumber = None          # mapping from LTL to structed english
        self.userAddedEnvLivenessEnglish = []          # keep track of liveness added by the user in English
        self.userAddedEnvLivenessLTL = []          # keep track of liveness added by the user in LTL
        self.originalLTLSpec      = {}          # save the original Spec for exporting
        self.LTLSpec  = {}
        self.sensor_strategy = None
        
        ############# NEW THING FOR THRESHOLDING FOR RESYNTHESIS
        self.envViolationCount = 0

        ################# WHAT MODE ARE WE IN
        self.recovery = False
        self.ENVcharacterization = True
        #########################################
        
        # -----------------------------------------#
        # -------- two_robot_negotiation ----------#
        # -----------------------------------------#
        self.robClient = None
        self.old_violated_specStr = []
        self.old_violated_specStr_with_no_specText_match = [] # for spec with no specText match
        self.old_violated_spec_line_no = []
        self.old_possible_states_violated_specStr = []
        self.old_possible_states_violated_specStr_with_no_specText_match = [] # for spec with no specText match
        self.old_possible_states_violated_spec_line_no = []
        self.prev_z  = str(0)
        # -----------------------------------------#

        # ********** patching ******************** #
        self.centralized_strategy_state = None # to save centralized strategy state during patching
        self.runCentralizedStrategy = False # track if we are running patching
        self.envTransCheck = None # for symbolic strategy checking
        self.sysTransCheck = None # for symbolic strategy checking
        # **************************************** #

        # %%%%%%%%%% d-patching %%%%%%%%%%%%%%%%% #
        self.dPatchingExecutor = None #executor of decentralized patching
        self.globalEnvTransCheck = None # violation check for global spec
        self.checkDataThread = None # main loop of dPatchingExecutor
        self.checkEnvTransViolationThread = None # for checking violations
        self.possibleStatesCheckEnvTransViolationThread = None # for checking violations
        # %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% #

    def setEnvViolationThreshold(self):
        if self.proj.compile_options['neighbour_robot'] and self.proj.compile_options['multi_robot_mode'] == 'negotiation':
            self.envViolationThres = 0 # currently for negotiation
        else:
            self.envViolationThres = 150 #100

    def postEvent(self, eventType, eventData=None):
        """ Send a notice that an event occurred, if anyone wants it """

        with self.postEventLock:
            if self.externalEventTarget is None:
                return

            try:
                self.externalEventTarget.handleEvent(eventType, eventData)
            except socket.error as e:
                ltlmop_logger.warning("Could not send event to remote event target: %s", e)
                ltlmop_logger.warning("Forcefully unsubscribing target.")
                self.externalEventTarget = None

    def loadSpecFile(self, filename):
        # Update with this new project
        self.proj = project.Project()
        self.proj.loadProject(filename)
        self.hsub = handlerSubsystem.HandlerSubsystem(self, self.proj.project_root)

        # update recovery status
        self.recovery = self.proj.compile_options["recovery"]

        # Tell GUI to load the spec file
        self.postEvent("SPEC", self.proj.getFilenamePrefix() + ".spec")

    def loadAutFile(self, filename):
        """
        This function loads the the .aut/.bdd file named filename and returns the strategy object.
        filename (string): name of the file with path included
        """
        enabled_sensors = self.proj.enabled_sensors

        if self.proj.rfi:
            if self.proj.compile_options["use_region_bit_encoding"]:
                sysRegions = copy.copy(self.proj.rfi.regions)
                if self.proj.compile_options["recovery"]:
                    # need to add dummy region as recovery allows extra environment bits
                    # Calculate the minimum number of bits necessary; note that we use max(1,...) because log(1)==0
                    num_props = max(1, int(math.ceil(math.log(len(self.proj.rfi.regions), 2))))

                    for x in range(2**num_props-len(self.proj.rfi.regions)):
                        sysRegions.append(None)

                region_domain = [strategy.Domain("region", sysRegions, strategy.Domain.B0_IS_MSB)]
            else:
                if self.proj.compile_options["decompose"]:
                    region_domain = [x.name for x in self.parser.proj.rfi.regions]
                else:
                    region_domain = [x.name for x in self.proj.rfi.regions if not x.isObstacle and x.name != 'boundary']
                    #region_domain = [strategy.Domain("region", region_domain)]
                    #ltlmop_logger.debug(region_domain[0].valueToPropAssignments('kitchen'))

            if self.proj.compile_options['fastslow'] :
                if self.proj.compile_options["use_region_bit_encoding"]:

                    envRegions = copy.copy(self.proj.rfi.regions)
                    if self.proj.compile_options["recovery"]:
                        # need to add dummy region as recovery allows extra environment bits

                        # Calculate the minimum number of bits necessary; note that we use max(1,...) because log(1)==0
                        num_props = max(1, int(math.ceil(math.log(len(self.proj.rfi.regions), 2))))

                        for x in range(2**num_props-len(self.proj.rfi.regions)):
                            envRegions.append(None)

                    regionCompleted_domain = [strategy.Domain("regionCompleted", envRegions, strategy.Domain.B0_IS_MSB)]
                    enabled_sensors = [x for x in self.proj.enabled_sensors if not x.endswith('_rc') or x.startswith(tuple(self.proj.otherRobot))]
                else:
                    regionCompleted_domain = []
                    enabled_sensors = [x for x in enabled_sensors if not x.endswith('_rc') or x.startswith(tuple(self.proj.otherRobot))]
                    if self.proj.compile_options["decompose"]:
                        enabled_sensors.extend([r.name+"_rc" for r in self.parser.proj.rfi.regions])
                    else:
                        enabled_sensors.extend([r.name+"_rc" for r in self.proj.rfi.regions])
            else:
                regionCompleted_domain = []
        else:
            region_domain = []
            regionCompleted_domain = []

        # figure out slugs option even if it's not used
        cmd = self.compiler._getSlugsCommand(execution=True)

        strat = strategy.createStrategyFromFile(self.proj.getStrategyFilename(),
                                                enabled_sensors  + regionCompleted_domain,
                                                self.proj.enabled_actuators + self.proj.all_customs  + self.proj.internal_props + region_domain, cmd)
        return strat

    def _getCurrentRegionFromPose(self, rfi=None):
        # TODO: move this to regions.py
        if rfi is None:
            rfi = self.proj.rfi

        pose = self.hsub.coordmap_lab2map(self.hsub.getPose())

        region = next((i for i, r in enumerate(rfi.regions) if r.name.lower() != "boundary" and \
                        r.objectContainsPoint(*pose)), None)

        if region is None:
            ltlmop_logger.warning("Pose of {} not inside any region!".format(pose))

        return region

    def shutdown(self):
        self.runStrategy.clear()
        ltlmop_logger.info("QUITTING.")

        all_handler_types = ['init', 'pose', 'locomotionCommand', 'drive', 'motionControl', 'sensor', 'actuator']
        # for htype in all_handler_types:
        #     ltlmop_logger.info("Terminating {} handler...".format(htype))
        #     if htype in self.proj.h_instance:
        #         if isinstance(self.proj.h_instance[htype], dict):
        #             handlers = [v for k,v in self.proj.h_instance[htype].iteritems()]
        #         else:
        #             handlers = [self.proj.h_instance[htype]]
        #         for h in handlers:
        #             if hasattr(h, "_stop"):
        #                 ltlmop_logger.debug("Calling _stop() on {}".format(h.__class__.__name__))
        #                 h._stop()
        #             else:
        #                 ltlmop_logger.debug("{} does not have _stop() function".format(h.__class__.__name__))
        #     else:
        #         ltlmop_logger.debug("{} handler not found in h_instance".format(htype))

        for h in self.hsub.handler_instance:
            ltlmop_logger.info("Terminating {} handler...".format(h.__class__.__name__))
            if hasattr(h, "_stop") and callable(getattr(h, "_stop")):
                ltlmop_logger.debug("Calling _stop() on {}".format(h.__class__.__name__))
                h._stop()
            else:
                ltlmop_logger.debug("{} does not have _stop() function".format(h.__class__.__name__))

        # ----------------------------- #
        # -- two_robot_negotiation  --- #
        # ----------------------------- #
        if self.robClient:
            self.robClient.closeConnection()
        # ----------------------------- #

        #%%%%%%%%%% d-patching %%%%%%%%%#
        if self.dPatchingExecutor:
            self.dPatchingExecutor.closeConnection(None,None)
        #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#
        self.alive.clear()

    def pause(self):
        """ pause execution of the automaton """
        self.runStrategy.clear()
        time.sleep(0.1) # Wait for FSA to stop
        self.postEvent("PAUSE")

    def resume(self):
        """ start/resume execution of the automaton """
        self.runStrategy.set()

    def isRunning(self):
        """ return whether the automaton is currently executing """
        return self.runStrategy.isSet()

    def registerExternalEventTarget(self, address):
        self.externalEventTarget = xmlrpclib.ServerProxy(address, allow_none=True)

        # Redirect all output to the log
        redir = RedirectText(self.externalEventTarget.handleEvent)

        sys.stdout = redir
        sys.stderr = redir

        self.externalEventTargetRegistered.set()

    def initialize(self, spec_file, strategy_file, firstRun=True):
        """
        Prepare for execution, by loading and initializing all the relevant files (specification, map, handlers, strategy)
        If `firstRun` is true, all handlers will be imported; otherwise, only the motion control handler will be reloaded.
        """
        self.runRuntimeMonitoring.clear() # pause runtime monitoring

        # load project only first time; otherwise self.proj is modified in-place
        # TODO: make this less hacky
        if firstRun:
            self.loadSpecFile(spec_file)

            if self.proj.compile_options['decompose']:
                self.proj.rfiold = self.proj.rfi  # Save the undecomposed regions

        self.setEnvViolationThreshold() # set different thres in different modes.

        if self.proj.compile_options['decompose']:
            self.proj.rfi = self.proj.loadRegionFile(decomposed=True)

        if self.proj.current_config == "":
            ltlmop_logger.error("Can not simulate without a simulation configuration.")
            ltlmop_logger.error("Please create one by going to [Run] > [Configure Simulation...] in SpecEditor and then try again.")
            sys.exit(2)

        ltlmop_logger.info("Setting current executing config...")
        self.hsub.setExecutingConfig(self.proj.current_config)

        # make sure the coord transformation function is ready
        # get the main robot config
        robot_config = self.hsub.executing_config.getRobotByName(self.hsub.executing_config.main_robot)
        self.hsub.coordmap_map2lab, self.hsub.coordmap_lab2map = robot_config.getCoordMaps()
        self.proj.coordmap_map2lab, self.proj.coordmap_lab2map = robot_config.getCoordMaps()

        ################self.proj.compile_options['neighbour_robot'] = False

        # Import the relevant handlers
        if firstRun:
            # Instantiate all handlers
            ltlmop_logger.info("Instantiate all handlers...")
            self.hsub.instantiateAllHandlers()

            ltlmop_logger.info("Preparing proposition mapping...")
            self.hsub.prepareMapping()

            # synthesize our controller again just to see if it's realizable and replace spec if FALSE
            self.compiler = specCompiler.SpecCompiler(spec_file)
            if self.proj.rfi and self.proj.compile_options['decompose']:
                self.compiler._decompose()  # WHAT DOES IT DO? DECOMPOSE REGIONS?
        else:
            #print "Reloading motion control handler..."
            #self.proj.importHandlers(['motionControl'])
            pass

        # We are done initializing at this point if there is no aut file yet
        if strategy_file is None:
            return

        # TODO: maybe an option for BDD here later
        # Load automaton file
        if self.proj.compile_options['symbolic']:
            self.strategy = None
        new_strategy = self.loadAutFile(strategy_file)

        if firstRun:
            ### Wait for the initial start command
            ltlmop_logger.info("Ready.  Press [Start] to begin...")
            self.runStrategy.wait()

        ######## ENV Assumption Learning ###########
        if firstRun:
            ###########
            #self.tracebackTree : separate spec lines to spec groups
            #############
            self.spec, self.tracebackTree, response = self.compiler._writeLTLFile()#False)
            self.originalLTLSpec  = self.spec.copy()
            realizable, realizableFS, output  = self.compiler._synthesize()

            # initializing dialog in simGUI when violation occurs
            self.simGUILearningDialog = ["Added current inputs", "Added current and incoming inputs", "Added current, incoming inputs and current outputs"]

            # for mapping from lineNo to LTL
            for key,value in self.compiler.LTL2SpecLineNumber.iteritems():
                self.LTLSpec[ value ] = key.replace("\t","").replace("\n","").replace(" ","")

            self.originalSpec = copy.deepcopy(self.spec)
            if not realizable:
                # start with always false
                self.oriEnvTrans = '[]((FALSE))' #added but should never be used for the unrealizable case.
                self.spec['EnvTrans'] = "\t[]((FALSE))\n"
                self.EnvTransRemoved = self.tracebackTree["EnvTrans"]
            else:
                # put all clauses in EnvTrans into conjuncts
                if self.proj.compile_options['fastslow']:
                    self.spec['EnvTrans'] = ' &\n'.join(filter(None, [self.spec['EnvTrans'], self.spec["EnvTopo"]]))
                self.oriEnvTrans = copy.copy(self.spec['EnvTrans'])
                self.spec['EnvTrans'] = '[](('+ copy.copy(self.oriEnvTrans).replace('[]','') +'))\n'
                self.EnvTransRemoved = []

            # rewrite ltl file
            self.recreateLTLfile(self.proj)

            # path of ltl file to be passed to the function
            self.path_LTLfile =  os.path.join(self.proj.project_root,self.proj.getFilenamePrefix()+".ltl")
            #create LTL checking object
            self.LTLViolationCheck = LTLParser.LTLcheck.LTL_Check(self.path_LTLfile,self.compiler.LTL2SpecLineNumber,self.spec)
            self.LTLViolationCheckPossibleStates = LTLParser.LTLcheck.LTL_Check(self.path_LTLfile,self.compiler.LTL2SpecLineNumber,self.spec)

            #safe a copy of the original sys initial condition (for resynthesis later)
            self.originalSysInit = self.spec['SysInit']

            # pass in current env assumptions if we have some
            if realizable:
                self.LTLViolationCheck.ltl_treeEnvTrans = LTLParser.LTLFormula.parseLTL(str(self.oriEnvTrans))
                #self.LTLViolationCheck.env_safety_assumptions_stage = {"1": self.spec['EnvTrans'][:-3] , "3": self.spec['EnvTrans'][:-3] , "2": self.spec['EnvTrans'][:-3] }
                self.LTLViolationCheckPossibleStates.ltl_treeEnvTrans = LTLParser.LTLFormula.parseLTL(str(self.oriEnvTrans))
            else:
                self.LTLViolationCheck.ltl_treeEnvTrans = None
                self.LTLViolationCheck.setOriginalEnvTrans('FALSE')

                self.LTLViolationCheckPossibleStates.ltl_treeEnvTrans = None
                self.LTLViolationCheckPossibleStates.setOriginalEnvTrans('FALSE')
        ############################################

        ### Figure out where we should start from by passing proposition assignments to strategy and search for initial state
        ### pass in sensor values, current actuator and custom proposition values, and current region object

        ## Region
        # FIXME: make getcurrentregion return object instead of number, also fix the isNone check
        if self.proj.rfi:
            init_region = self.proj.rfi.regions[self._getCurrentRegionFromPose()]
            if init_region is None:
                ltlmop_logger.error("Initial pose not inside any region!")
                sys.exit(-1)

            ltlmop_logger.info("Starting from initial region: " + init_region.name)
            # include initial regions in picking states
            if self.proj.compile_options['fastslow']:
                init_prop_assignments = {"regionCompleted": init_region}
                # TODO: check init_region format
            else:
                init_prop_assignments = {"region": init_region}
        else:
            init_prop_assignments = {}
        ## outputs
        if firstRun or self.strategy is None:
            # initialize all sensor and actuator methods
            ltlmop_logger.info("Initializing sensor and actuator methods...")
            self.hsub.initializeAllMethods()

            # save the initial values of the actuators and the custom propositions
            for prop in self.proj.enabled_actuators + self.proj.all_customs:
                self.current_outputs[prop] = (prop in self.hsub.executing_config.initial_truths)

        init_prop_assignments.update(self.current_outputs)


        if self.proj.compile_options['neighbour_robot']:
            if self.proj.compile_options["multi_robot_mode"] == "patching" or self.proj.compile_options["multi_robot_mode"] == "negotiation":
                # -------- two_robot_negotiation ----------#
                if firstRun:
                    self.robClient = negotiationMonitor.robotClient.RobotClient(self.hsub, self.proj)
                self.robClient.updateRobotRegion(self.proj.rfi.regions[self._getCurrentRegionFromPose()])
                tempDict = self.hsub.getSensorValue([x for x in self.proj.enabled_sensors\
                    if not x.startswith(tuple(self.proj.otherRobot)) and not x.endswith('_rc')])
                tempDict.update(self.current_outputs)
                self.robClient.updateRobotSensors(tempDict) # also update sensors
                if self.proj.compile_options['include_heading']:
                    self.robClient.updateCompletedRobotRegion(self.proj.rfi.regions[self._getCurrentRegionFromPose()])
                # check negotiation statue only for two robot nego
                if self.proj.compile_options["multi_robot_mode"] == "negotiation":
                    self.negotiationStatus = self.robClient.checkNegotiationStatus()
                # -----------------------------------------#
            elif self.proj.compile_options["multi_robot_mode"] == "d-patching":
                # %%%%%%%% d-patching %%%%%%%%% #
                if firstRun:
                    if self.proj.compile_options['include_heading']:
                        self.dPatchingExecutor = centralCoordinator.decentralizedPatchingExecutor.PatchingExecutor(self.hsub, self.proj, \
                            self.proj.rfi.regions[self._getCurrentRegionFromPose()], self.proj.rfi.regions[self._getCurrentRegionFromPose()])
                    else:
                        self.dPatchingExecutor = centralCoordinator.decentralizedPatchingExecutor.PatchingExecutor(self.hsub, self.proj, \
                            self.proj.rfi.regions[self._getCurrentRegionFromPose()])

                    # set global sensors
                    self.dPatchingExecutor.globalSensors = self.proj.global_sensors
                    ltlmop_logger.debug('self.dPatchingExecutor.globalSensors:' + str(self.dPatchingExecutor.globalSensors))
                    self.checkDataThread = threading.Thread(target=self.dPatchingExecutor.runCheckData, args=())
                    self.checkDataThread.daemon = True  # Daemonize thread
                    self.checkDataThread.start()
                # %%%%%%%%%%%%%%%%%%%%%%%%%%%%% #
            else:
                ltlmop_logger.error('You have selected the neighbour_robot option but the multi_robot_mode should be defined!')
                sys.exit(3)

        # update GUI
        pose = self.hsub.getPose(cached=True)[0:2]
        self.postEvent("POSE", tuple(map(int, self.hsub.coordmap_lab2map(pose))))

        ## inputs
        # ---- two_robot_negotiation ----- #
        if self.proj.compile_options['neighbour_robot']:

            if firstRun:
                # set up dictionary tracking negotiations
                for robot in self.proj.otherRobot:
                    self.receivedSpec[robot] = False  #track if we have recevied request from the other robot
                    self.sentSpec[robot] = False  #track if we have sent spec to the other robot
                    self.exchangedSpec[robot] = False  #track if we have exchanged spec with the other robot

            # Wait until the other robot is ready
            # Make sure the other robot is loaded
            ltlmop_logger.info('Waiting for other robots to be ready')
            otherRobotsReady = False

            while not otherRobotsReady:
                ltlmop_logger.debug('Current Sensors:' + str(self.hsub.getSensorValue(self.proj.enabled_sensors)))
                if not None in self.hsub.getSensorValue(self.proj.enabled_sensors).values():
                    otherRobotsReady = True
                time.sleep(2)
        # -------------------------------- #

        if self.proj.rfi:
            if self.proj.compile_options['fastslow']:
                init_prop_assignments.update(self.hsub.getSensorValue([x for x in self.proj.enabled_sensors if not x.endswith('_rc') or x.startswith(tuple(self.proj.otherRobot))]))
            else:
                init_prop_assignments.update(self.hsub.getSensorValue(self.proj.enabled_sensors))

        #search for initial state in the strategy
        if firstRun:
            ltlmop_logger.debug("init_prop_assignments:" + str(init_prop_assignments))
            init_state = new_strategy.searchForOneState(init_prop_assignments)
        else:
            # ------- patching ----------#
            if self.proj.compile_options['neighbour_robot'] and \
                (self.proj.compile_options["multi_robot_mode"] == "patching" or self.proj.compile_options["multi_robot_mode"] == "d-patching"):
                if not self.spec['SysGoals'].count('[]<>') == 1: # LTLParser doesn't parse single formula with []<> correctly.
                    specLen = len(LTLParser.LTLcheck.ltlStrToList(self.spec['SysGoals']))
                    ltlmop_logger.debug('Old goal number is:' + str(self.prev_z))
                    current_goal_id = str((int(self.prev_z) + 1) % specLen)
                else:
                    current_goal_id = str(0)

                # also add in the system as that's what make winPos satisfied
                if self.dPatchingExecutor.strategy.current_state:
                    centralizedSysPropDict = {k:v for k, v in self.dPatchingExecutor.strategy.current_state.getOutputs(expand_domains=True).iteritems() if self.dPatchingExecutor.robotName in k}
                    for propKey, propValue in centralizedSysPropDict.iteritems():
                        if propKey.replace(self.dPatchingExecutor.robotName+'_','') in self.proj.regionMapping.keys():
                            if propValue: #regionProp
                                init_prop_assignments.update({"region": \
                                                              self.proj.rfi.regions[self.proj.rfi.indexOfRegionWithName(self.proj.regionMapping[propKey.replace(self.dPatchingExecutor.robotName+'_','')][0])]\
                                                            })
                        else: # normal prop
                            init_prop_assignments.update({propKey.replace(self.dPatchingExecutor.robotName+'_',''): propValue})
                # ---------------------------- #
            else:
                current_goal_id = self.prev_z

            ltlmop_logger.debug('init_prop_assignments:' + str(init_prop_assignments))
            ltlmop_logger.debug("Current goal number is:" + current_goal_id)

            init_state = new_strategy.searchForOneState(init_prop_assignments, goal_id=current_goal_id)

        #for using get LTLRepresentation of current sensors
        self.sensor_strategy = new_strategy.states.addNewState()

        # resynthesize if cannot find initial state
        if init_state is None: 

            # check if execution is paused
            if not self.runStrategy.isSet():
                self.stopMotionAndAction()

                ###### ENV VIOLATION CHECK ######
                # pop up the analysis dialog
                #self.onMenuAnalyze(enableResynthesis = False, exportSpecification = True)
                ################################

                # wait for either the FSA to unpause or for termination
                while (not self.runStrategy.wait(0.1)) and self.alive.isSet():
                    pass


            ltlmop_logger.debug('Finding init state failed.')
            for prop_name, value in self.hsub.getSensorValue(self.proj.enabled_sensors).iteritems():
                if self.proj.compile_options['fastslow'] and prop_name.endswith('_rc') and not prop_name.startswith(tuple(self.proj.otherRobot)):
                    continue
                self.sensor_strategy.setPropValue(prop_name, value)

            if self.proj.compile_options['fastslow']:
                self.sensor_strategy.setPropValue("regionCompleted", self.proj.rfi.regions[self._getCurrentRegionFromPose()])

            self.postEvent('INFO','Finding init state failed.')
            # store time stamp of violation
            if self.proj.compile_options["neighbour_robot"]:
                if self.proj.compile_options["multi_robot_mode"] == "negotiation":
                    if not self.disableEnvChar:
                        if self.violationTimeStamp == 0:
                            self.violationTimeStamp = time.clock()
                            self.robClient.setViolationTimeStamp(self.violationTimeStamp)
                            ltlmop_logger.debug('Setting violation timeStamp')
                            time.sleep(1)

            init_state, new_strategy  = self.addStatetoEnvSafety(self.sensor_strategy, firstRun)            
        #############################################
        if init_state is None:
            ltlmop_logger.error("No suitable initial state found; unable to execute. Quitting...")
            sys.exit(-1)
        else:
            ltlmop_logger.info("Starting from state %s." % init_state.state_id)
            if self.strategy is None or init_state.state_id != self.strategy.current_state.state_id:
                self.postEvent('INFO', "Starting from state %s." % init_state.state_id)

        self.strategy = new_strategy
        self.strategy.current_state = init_state
        ltlmop_logger.debug("self.strategy.current_state:" + str(self.strategy.current_state))
        self.last_sensor_state = self.strategy.current_state.getInputs()

        if self.proj.compile_options['symbolic'] or self.proj.compile_options['interactive']:
            self.envTransCheck = LTLParser.LTLcheck.LTL_Check(None,{}, self.spec, 'EnvTrans')
            self.sysTransCheck = LTLParser.LTLcheck.LTL_Check(None,{}, self.spec, 'SysTrans')
            if self.proj.compile_options['symbolic']:
                ltlmop_logger.debug('We came here')
                self.strategy.envTransBDD, term1, term2 = self.strategy.evaluateBDD(self.envTransCheck.ltl_tree, LTLParser.LTLFormula.p.terminals)
                ltlmop_logger.debug('We finished')

        # start checkViolation thread
        if not self.proj.compile_options['neighbour_robot'] or not self.proj.compile_options["multi_robot_mode"] == "negotiation":
            #self.checkEnvTransViolationThread = threading.Thread(target=self.run_check_envTrans_violations, args=())
            self.checkEnvTransViolationThread = threading.Thread(target=self.run_simple_check_envTrans_violations, args=())
            self.checkEnvTransViolationThread.daemon = True  # Daemonize thread
            self.checkEnvTransViolationThread.start()

            #self.possibleStatesCheckEnvTransViolationThread = threading.Thread(target=self.run_check_envTrans_violations, args=())
            #self.possibleStatesCheckEnvTransViolationThread.daemon = True  # Daemonize thread
            #self.possibleStatesCheckEnvTransViolationThread.start()
            self.runRuntimeMonitoring.set() # start checking violations

        # set up check for goals
        if not self.spec['SysGoals'].count('[]<>') == 1:
            self.sysGoalsList  = LTLParser.LTLcheck.ltlStrToList(self.spec['SysGoals'])
        else:
            self.sysGoalsList =[self.spec['SysGoals']]
        self.LTLViolationCheckSysGoalslist = []
        for ltlStr in self.sysGoalsList:
            self.LTLViolationCheckSysGoalslist.append(LTLParser.LTLcheck.LTL_Check("",{},{'SysGoals':ltlStr},specType='SysGoals'))
        self.goals_checker = False

        return  init_state, self.strategy

    def stopMotionAndAction(self):
        # this funciton stops motion and action currently executing
        self.hsub.setVelocity(0,0)

        # pause action as well
        for h in self.hsub.handler_instance:
            ltlmop_logger.log(8, str(h.__class__) + str(isinstance(h,ht.ActuatorHandler)))
            if isinstance(h,ht.ActuatorHandler):
                ltlmop_logger.log(8, getattr(h, "_pause", None))
                pauseFn = getattr(h, "_pause", None)
                if callable(pauseFn):
                    h._pause()
        #self.postEvent("INFO","Pause all motion and actions")

    def resumeMotionAndAction(self):
        # restart action
        for h in self.hsub.handler_instance:
            ltlmop_logger.log(8, str(h.__class__) + str(isinstance(h,ht.ActuatorHandler)))
            if isinstance(h,ht.ActuatorHandler):
                ltlmop_logger.log(8, getattr(h, "_resume", None))
                pauseFn = getattr(h, "_resume", None)
                if callable(pauseFn):
                    h._resume()
        #self.postEvent("INFO","Resume all motion and actions")

    def run(self):
        ### Get everything moving
        # Rate limiting is approximately 20Hz
        avg_freq = 20
        last_gui_update_time = 0

        # track number of goals
        if not self.spec['SysGoals'].count('[]<>') == 1:
            self.totalSysGoals = len(LTLParser.LTLcheck.ltlStrToList(self.spec['SysGoals']))
        else:
            self.totalSysGoals = 1
        ltlmop_logger.debug("totalSysGoals:" + str(self.totalSysGoals))

        # FIXME: don't crash if no spec file is loaded initially
        while self.alive.isSet():
            # Idle if we're not running
            if not self.runStrategy.isSet():
                self.stopMotionAndAction()

                ###### ENV VIOLATION CHECK ######
                # pop up the analysis dialog                
                #self.onMenuAnalyze(enableResynthesis = False, exportSpecification = True)
                ################################
                    
                # wait for either the FSA to unpause or for termination
                while (not self.runStrategy.wait(0.1)) and self.alive.isSet():
                    pass

                self.resumeMotionAndAction()

            # Exit immediately if we're quitting
            if not self.alive.isSet():
                break

            self.prev_outputs = self.strategy.current_state.getOutputs()

            tic = self.timer_func()
            ###### ENV VIOLATION CHECK ######  
            last_next_states = self.last_next_states

            #check if goal is satisfied
            if self.proj.compile_options['fastslow']:
                goal_id = int(self.prev_z)
                deepcopy_sensor_state = copy.deepcopy(self.sensor_strategy)
                deepcopy_current_state = copy.deepcopy(self.strategy.current_state)
                if self.LTLViolationCheckSysGoalslist[goal_id].checkViolation(deepcopy_current_state, deepcopy_sensor_state):
                    if not self.goals_checker:
                        ltlmop_logger.info("+++++++++++++")
                        ltlmop_logger.info("System Goal {0}: {1} is True.".format(self.prev_z, self.sysGoalsList[goal_id]))
                        ltlmop_logger.info("+++++++++++++")
                    self.goals_checker = True
                else:
                    self.goals_checker = False

            ####################################
            ###### RUN STRATEGY ITERATION ######
            ####################################

            if not self.proj.compile_options['fastslow']:
                self.runStrategyIteration()
            else:
                if self.proj.compile_options['neighbour_robot'] and self.proj.compile_options["multi_robot_mode"] == "patching" and self.robClient.getCentralizedExecutionStatus():
                    # *********** patching ************** #
                    # running centralized strategy
                    if not self.runCentralizedStrategy:
                        self.runCentralizedStrategy = True
                        self.postEvent("PATCH","Start running centralized strategy ...")

                    # first check if global system goals are satisfied
                    # if self.dPatchingExecutor.goalsSatisfied:
                    #     self.hsub.setVelocity(0,0)
                    self.runStrategyIterationInstanteousActionCentralized()
                    # *********************************** #

                elif self.proj.compile_options['neighbour_robot'] and self.proj.compile_options["multi_robot_mode"] == "d-patching" \
                    and self.dPatchingExecutor.getCentralizedExecutionStatus():
                    #%%%%%%%% d-patching %%%%%%%%%%%#
                    # running centralized strategy
                    if not self.runCentralizedStrategy:
                        self.runCentralizedStrategy = True
                        self.postEvent("D-PATCH","Start running centralized strategy ...")

                    # first check if global system goals are satisfied
                    # if self.dPatchingExecutor.goalsSatisfied:
                    #     self.hsub.setVelocity(0,0)

                    # update info from other robots
                    self.dPatchingExecutor.runIterationCentralExecution()

                    self.runStrategyIterationInstanteousActionCentralized()
                    #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#

                else:
                    if self.runCentralizedStrategy:
                        if self.proj.compile_options['neighbour_robot'] and self.proj.compile_options["multi_robot_mode"] == "patching":
                            # *********** patching ************** #
                            # once switched back to local strategy need to find init state again
                            self.runCentralizedStrategy = False
                            self.postEvent("PATCH","Centralized strategy ended. Resuming local strategy ...")
                            self.stopMotionAndAction()
                            self.runRuntimeMonitoring.clear()
                            spec_file = self.proj.getFilenamePrefix() + ".spec"
                            aut_file = self.proj.getFilenamePrefix() + ".aut"
                            self.initialize(spec_file, aut_file, firstRun=False)
                            self.robClient.loadProjectAndRegions(self.proj) #update regions and proj in robClient
                            self.robClient.setRestartStatus()
                            while not self.robClient.checkRestartStatus():
                                ltlmop_logger.debug('Waiting for the other robot to restart')
                                time.sleep(1) #wait for the other robot to get ready
                            ltlmop_logger.debug('Running again ...')
                            self.resumeRuntimeMonitoring()
                            self.resumeMotionAndAction()
                            continue
                            # *********************************** #
                        elif self.proj.compile_options['neighbour_robot'] and self.proj.compile_options["multi_robot_mode"] == "d-patching":
                            # %%%%%%%%%%% d-patching  %%%%%%%%%%% #
                            # once switched back to local strategy need to find init state again
                            # also stops the other robots operating
                            for robot in self.dPatchingExecutor.robotInRange:
                                self.dPatchingExecutor.setPauseForControllerSynthesis(self.dPatchingExecutor.clients[robot], True)

                            self.postEvent("D-PATCH","Centralized strategy ended. Resuming local strategy ...")
                            self.runCentralizedStrategy = False
                            self.stopMotionAndAction()
                            self.runRuntimeMonitoring.clear()
                            ltlmop_logger.debug("stopped runRuntimeMonitoring...")
                            spec_file = self.proj.getFilenamePrefix() + ".spec"
                            aut_file = self.proj.getFilenamePrefix() + ".aut"
                            self.initialize(spec_file, aut_file, firstRun=False)
                            ltlmop_logger.debug("finished initializing")
                            self.dPatchingExecutor.sendRestartStatusToAllCoordinatingRobots()

                            # updated regions
                            self.dPatchingExecutor.proj = self.proj
                            self.dPatchingExecutor.loadProjectAndRegions()

                            while not self.dPatchingExecutor.checkRestartStatus():
                                ltlmop_logger.debug('Waiting for the other robot to restart')
                                self.dPatchingExecutor.sendRestartStatusToAllCoordinatingRobots()
                                self.dPatchingExecutor.runIterationNotCentralExecution()
                                ltlmop_logger.debug("RestartStatus:" + str(self.dPatchingExecutor.readyToRestart))
                                time.sleep(0.2) #wait for the other robot to get ready
                            ltlmop_logger.debug('Running again ...')
                            self.resumeRuntimeMonitoring()
                            self.resumeMotionAndAction()

                            # also restarts the other robots operating
                            for robot in self.dPatchingExecutor.robotInRange:
                                self.dPatchingExecutor.setPauseForControllerSynthesis(self.dPatchingExecutor.clients[robot], False)
                            continue
                            # %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#
                        else:
                            ltlmop_logger.warning('runCentralizedStrategy should not be true!')

                    # %%%%%%%%%%% d-patching  %%%%%%%%%%% #
                    # get latest info from the other robots.
                    if self.proj.compile_options['neighbour_robot'] and self.proj.compile_options["multi_robot_mode"] == "d-patching":
                        self.dPatchingExecutor.runIterationNotCentralExecution()
                    # %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#

                    self.runStrategyIterationInstanteousAction()

            #######################################
            ########## UPDATE GOAL ID #############
            #######################################
            #also make sure we are not in centralized mode to change goal_id
            if self.strategy.current_state.goal_id and int(self.strategy.current_state.goal_id) < self.totalSysGoals and not self.runCentralizedStrategy:
                self.prev_z = self.strategy.current_state.goal_id
            else:
                pass #stays the same

            if not (self.proj.compile_options['neighbour_robot'] and self.proj.compile_options["multi_robot_mode"] == "patching" and self.robClient.getCentralizedExecutionStatus()):
                # set current_next_states to compare with last_next_states
                current_next_states = self.last_next_states

                #############################################
                ######### CHECK ENVTRANS VIOLATION ##########
                #############################################
                # note that for negotiation: env_assumption_hold is updated in executeStrategy
                #           for patching: env_assumption hold is updated in thread
                env_assumption_hold = self.env_assumption_hold and self.possible_states_env_assumption_hold #self.check_envTrans_violations() #

                if not env_assumption_hold and not self.proj.compile_options['recovery']:
                    self.stopMotionAndAction()

                ###############################################################
                ####### CHECK IF REQUEST FROM OTHER ROBOTS IS RECEVIED ########
                ###############################################################
                if self.proj.compile_options['neighbour_robot']:
                    if self.proj.compile_options["multi_robot_mode"] == "negotiation":
                        # ---------- two_robot_negotiation ------------- #
                        # resynthesis request from the other robot
                        if self.receiveRequestFromEnvRobot():
                            self.resumeMotionAndAction()
                            continue
                        # ---------------------------------------------- #

                    elif self.proj.compile_options["multi_robot_mode"] == "patching":
                        # ************ patching ****************** #
                        # check if centralized strategy is initialized for the current robot (also make sure the spec is only sent until centralized execution starts)
                        if self.robClient.checkCoordinationRequest() and not self.robClient.getCentralizedExecutionStatus():
                            # stop the robot from moving
                            self.stopMotionAndAction()
                            self.postEvent("PATCH","We are asked to join a centralized strategy")
                            self.runRuntimeMonitoring.clear()
                            self.initiatePatching()
                            self.resumeRuntimeMonitoring()
                            self.resumeMotionAndAction()
                            # jump to top of while loop
                            continue
                        # **************************************** #
                    elif self.proj.compile_options["multi_robot_mode"] == "d-patching":
                        # %%%%%%%%%%%%% d-patching %%%%%%%%%%%%%%%% #
                        #TODO: !!!! REcheck.  if any other robots asked for coorindation
                        if not self.dPatchingExecutor.checkIfOtherRobotsAreReadyToExecuteCentralizedStrategy() and self.dPatchingExecutor.checkIfCoordinationRequestIsRecevied():
                            # stop the robot from moving
                            self.stopMotionAndAction()
                            self.postEvent("D-PATCH","We are asked to join a centralized strategy")
                            ltlmop_logger.info("We are asked to join a centralized strategy")
                            self.runRuntimeMonitoring.clear()
                            if self.runCentralizedStrategy:
                                self.initiateDPatchingCentralizedMode(received_request=True)
                            else:
                                self.initiateDPatching(received_request=True)
                            self.resumeRuntimeMonitoring()
                            self.resumeMotionAndAction()
                            #TODO: need to take care of cases where mulptiple requests are received
                            ltlmop_logger.error('Decentralized Patching is not completed yet!')
                            ltlmop_logger.info("Start executing centralized strategy")
                            continue
                        # %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% #
                    else:
                        ltlmop_logger.error("Mulit robot mode is incorrect. This is impossible.")

                #######################################################
                ######### ASSUMPTIONS DIDN'T HOLD ACTIONS #############
                #######################################################
                if not env_assumption_hold and self.runCentralizedStrategy:
                    if self.proj.compile_options["neighbour_robot"]:
                        if self.proj.compile_options["multi_robot_mode"] == "d-patching":
                            # %%%%%%%%%%% d-patching %%%%%%%%%%%% #
                            # centralized. show violated spec
                            self.postEvent("VIOLATION", "Detected the following env safety violation of global spec:")
                            for x in self.violated_spec_list:
                                if x not in self.old_violated_specStr:
                                    self.postEvent("VIOLATION", x)
                            self.old_violated_specStr = self.violated_spec_list

                            for x in self.possible_states_violated_spec_list:
                                if x not in self.old_possible_states_violated_specStr and x not in self.old_violated_specStr:
                                    self.postEvent("VIOLATION", "POSSIBLE violation:" + str(x))
                            self.old_possible_states_violated_specStr = self.possible_states_violated_spec_list
                            # stop the robot from moving
                            self.stopMotionAndAction()

                            # Take care of everything to start patching
                            #self.postEvent("RESOLVED", "")
                            self.postEvent("D-PATCH", "We will modify the spec or ask other robots for help.")
                            self.runRuntimeMonitoring.clear()
                            self.initiateDPatchingCentralizedMode()
                            self.resumeRuntimeMonitoring()
                            self.resumeMotionAndAction()
                            self.postEvent("D-PATCH","Resuming centralized strategy ...")
                            ltlmop_logger.warning("centralized repatch done... restarting")
                            # %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% #

                        elif self.proj.compile_options["multi_robot_mode"] == "patching":
                            # ************ patching ****************** #
                            # PATCHING CODE NEEDED
                            pass
                            # **************************************** #

                elif not env_assumption_hold and not self.runCentralizedStrategy:
                    if self.proj.compile_options['neighbour_robot']:
                        if self.proj.compile_options["multi_robot_mode"] == "negotiation":
                            # ------------ two_robot_negotiation ----------#
                            # store time stamp of violation
                            if not self.disableEnvChar:
                                if self.violationTimeStamp == 0:
                                    self.violationTimeStamp = time.clock()
                                    self.robClient.setViolationTimeStamp(self.violationTimeStamp)
                                    ltlmop_logger.debug('Setting violation timeStamp')
                                    time.sleep(1)
                            # ---------------------------------------------- #

                        elif self.proj.compile_options["multi_robot_mode"] == "patching":
                            # ************ patching ****************** #
                            # PATCHING CODE NEEDED
                            pass
                            # **************************************** #

                        elif self.proj.compile_options["multi_robot_mode"] == "d-patching":
                            # %%%%%%%%%%%%% d-patching %%%%%%%%%%%%%%%% #
                            ltlmop_logger.error('Decentralized Patching is not completed yet!')
                            pass
                            # %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% #

                        else:
                            ltlmop_logger.error("Mulit robot mode is incorrect. This is impossible.")

                    # count the number of next state changes
                    if last_next_states != current_next_states or str(self.strategy.current_state.state_id) not in [x.state_id for x in last_next_states]:
                        self.envViolationCount += 1
                        ltlmop_logger.debug("No of env violations:"+ str(self.envViolationCount))
                        if self.proj.compile_options['recovery']:
                            self.runRuntimeMonitoring.set()

                    # print out the violated specs
                    for x in self.violated_spec_line_no:
                        if x not in self.old_violated_spec_line_no:
                            if x == 0 :
                                if len(self.violated_spec_line_no) == 1 and len(self.old_violated_spec_line_no) == 0:
                                    self.postEvent("VIOLATION","Detected violation of env safety from env characterization")
                            else:
                                self.postEvent("VIOLATION","Detected the following env safety violation:" )
                                self.postEvent("VIOLATION", str(self.proj.specText.split('\n')[x-1]))

                    for x in self.violated_spec_list_with_no_specText_match:
                        if x not in self.old_violated_specStr_with_no_specText_match:
                            self.postEvent("VIOLATION","Detected the following violation:")
                            self.postEvent("VIOLATION", x)

                    # print out the violated possible specs
                    for x in self.possible_states_violated_spec_line_no:
                        if x not in list(set(self.old_possible_states_violated_spec_line_no + self.old_violated_spec_line_no)):
                            if x == 0 :
                                if len(self.possible_states_violated_spec_line_no) == 1 and len(self.old_possible_states_violated_spec_line_no) == 0:
                                    self.postEvent("VIOLATION","Detected violation of env safety from env characterization")
                            else:
                                self.postEvent("VIOLATION","Detected the following env safety violation:" )
                                self.postEvent("VIOLATION", "POSSIBLE violation: " + str(self.proj.specText.split('\n')[x-1]))

                    for x in self.possible_states_violated_spec_list_with_no_specText_match:
                        if x not in list(set(self.old_possible_states_violated_specStr_with_no_specText_match + self.old_violated_specStr_with_no_specText_match)):
                            self.postEvent("VIOLATION","Detected the following violation:")
                            self.postEvent("VIOLATION", "POSSIBLE violation:" + str(x))

                    # save a copy
                    self.old_violated_specStr = copy.copy(self.violated_spec_list)
                    self.old_violated_specStr_with_no_specText_match = copy.copy(self.violated_spec_list_with_no_specText_match)
                    self.old_violated_spec_line_no = copy.copy(self.violated_spec_line_no)

                    self.old_possible_states_violated_specStr_with_no_specText_match = copy.copy(self.possible_states_violated_spec_list_with_no_specText_match)
                    self.old_possible_states_violated_spec_line_no = copy.copy(self.possible_states_violated_spec_line_no)

                    if self.proj.compile_options['neighbour_robot'] and self.proj.compile_options["multi_robot_mode"] == "patching":
                        # ******* patching ********** #
                        # stop the robot from moving
                        self.stopMotionAndAction()

                        # Take care of everything to start patching
                        self.postEvent("RESOLVED","")
                        self.postEvent("PATCH","We will now ask for a centralized strategy to be executed.")
                        self.runRuntimeMonitoring.clear()
                        self.initiatePatching()
                        self.resumeRuntimeMonitoring()
                        self.resumeMotionAndAction()
                        continue
                        # *************************** #
                    elif self.proj.compile_options['neighbour_robot'] and self.proj.compile_options["multi_robot_mode"] == "d-patching":
                        # %%%%%%%%%%% d-patching %%%%%%%%%%%% #
                                                # stop the robot from moving
                        self.stopMotionAndAction()

                        # Take care of everything to start patching
                        self.postEvent("RESOLVED","")
                        self.runRuntimeMonitoring.clear()
                        ltlmop_logger.info("Initiating patching...")
                        self.initiateDPatching()
                        self.resumeRuntimeMonitoring()
                        self.resumeMotionAndAction()
                        continue
                        # %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% #
                    else:
                        if self.ENVcharacterization:
                            if self.recovery:
                                ########################################
                                #### FOR BOTH LEANRING AND RECOVERY  ###
                                ########################################

                                if str(self.strategy.current_state.state_id) in [x.state_id for x in last_next_states] \
                                or self.envViolationCount >= self.envViolationThres:

                                    # reset next state difference count
                                    self.envViolationCount = 0

                                    # print out assumptions violated
                                    for x in self.LTLViolationCheck.violated_spec_line_no:
                                        if x != 0:
                                            if x not in self.EnvTransRemoved:
                                                self.EnvTransRemoved.append(x)


                                    # stop the robot from moving ## needs testing again
                                    self.stopMotionAndAction()

                                    # Modify the ltl file based on the enviornment change
                                    self.addStatetoEnvSafety(self.sensor_strategy)

                                    # remove line numbers denoted as violated
                                    for x in self.EnvTransRemoved:
                                        if x in self.LTLViolationCheck.violated_spec_line_no:
                                            self.LTLViolationCheck.violated_spec_line_no.remove(x)

                            else:
                                ###################################
                                ####### FOR ONLY LEANRING #########
                                ###################################
                                # stop the robot from moving
                                self.stopMotionAndAction()

                                # Modify the ltl file based on the enviornment change
                                self.addStatetoEnvSafety(self.sensor_strategy)

                else:
                    # assumption not violated but sensor state changes. we add in this new state
                    #self.LTLViolationCheck.append_state_to_LTL(self.strategy.current_state, self.sensor_strategy)

                    if env_assumption_hold == False:
                        ltlmop_logger.debug("Value should be True: " + str(env_assumption_hold))

                    # For print violated safety in the log (update lines violated in every iteration)
                    if len(self.violated_spec_line_no) == 0 and self.old_violated_spec_line_no !=self.violated_spec_line_no and (self.recovery or self.disableEnvChar):
                        if not self.proj.compile_options['neighbour_robot']:
                            self.postEvent("RESOLVED", "The specification violation is resolved.")
                            #self.old_violated_spec_line_no = copy.copy(self.violated_spec_line_no)

                        # ------------ two_robot_negotiation ----------#
                        # store time stamp of violation
                        if self.proj.compile_options['neighbour_robot'] and self.proj.compile_options["multi_robot_mode"] == "negotiation":
                            self.postEvent("RESOLVED", "The specification violation is resolved.")
                            self.violationTimeStamp = 0
                            self.robClient.setViolationTimeStamp(self.violationTimeStamp)
                            ltlmop_logger.debug('Resetting violation timeStamp')

                            # # save a copy
                            # self.old_violated_specStr = copy.copy(self.violated_spec_list)
                            # self.old_violated_specStr_with_no_specText_match = copy.copy(self.violated_spec_list_with_no_specText_match)
                            # self.old_violated_spec_line_no = copy.copy(self.violated_spec_line_no)

                            # self.old_possible_states_violated_specStr_with_no_specText_match = copy.copy(self.possible_states_violated_spec_list_with_no_specText_match)
                            # self.old_possible_states_violated_spec_line_no = copy.copy(self.possible_states_violated_spec_line_no)

                    # save a copy
                    self.old_violated_specStr = copy.copy(self.violated_spec_list)
                    self.old_violated_specStr_with_no_specText_match = copy.copy(self.violated_spec_list_with_no_specText_match)
                    self.old_violated_spec_line_no = copy.copy(self.violated_spec_line_no)

                    self.old_possible_states_violated_specStr_with_no_specText_match = copy.copy(self.possible_states_violated_spec_list_with_no_specText_match)
                    self.old_possible_states_violated_spec_line_no = copy.copy(self.possible_states_violated_spec_line_no)
                        # ---------------------------------------------- #

                #self.currentViolationLineNo = self.LTLViolationCheck.violated_spec_line_no[:]

            #################################
            
            toc = self.timer_func()

            #self.checkForInternalFlags()

            # Rate limiting of execution and GUI update
            while (toc - tic) < 0.05:
                time.sleep(0.005)
                toc = self.timer_func()

            # Update GUI
            # If rate limiting is disabled in the future add in rate limiting here for the GUI:
            # if show_gui and (timer_func() - last_gui_update_time > 0.05)
            avg_freq = 0.9 * avg_freq + 0.1 * 1 / (toc - tic) # IIR filter
            self.postEvent("FREQ", int(math.ceil(avg_freq)))
            pose = self.hsub.getPose(cached=True)[0:2]
            self.postEvent("POSE", tuple(map(int, self.hsub.coordmap_lab2map(pose))))

            last_gui_update_time = self.timer_func()

        ltlmop_logger.debug("execute.py quitting...")

    # This function is necessary to prevent xmlrpcserver from catching
    # exceptions and eating the tracebacks
    def _dispatch(self, method, args):
        try:
            return getattr(self, method)(*args)
        except:
            traceback.print_exc()
            raise

class RedirectText:
    def __init__(self, event_handler):
        self.event_handler = event_handler

    def write(self, message):
        if message.strip() != "":
            self.event_handler("OTHER", message.strip())

    def flush(self):
        pass


####################################################
# Main function, run when called from command-line #
####################################################

def execute_main(listen_port=None, spec_file=None, aut_file=None, show_gui=False):
    ltlmop_logger.info("Hello. Let's do this!")

    # Create the XML-RPC server
    if listen_port is None:
        # Search for a port we can successfully bind to
        while True:
            listen_port = random.randint(10000, 65535)
            try:
                xmlrpc_server = SimpleXMLRPCServer(("127.0.0.1", listen_port), logRequests=False, allow_none=True)
            except socket.error as e:
                pass
            else:
                break
    else:
        xmlrpc_server = SimpleXMLRPCServer(("127.0.0.1", listen_port), logRequests=False, allow_none=True)

    try:
        # Create the execution context object
        e = LTLMoPExecutor()

        # Register functions with the XML-RPC server
        xmlrpc_server.register_instance(e)

        # Kick off the XML-RPC server thread
        XMLRPCServerThread = threading.Thread(target=xmlrpc_server.serve_forever)
        XMLRPCServerThread.daemon = True
        XMLRPCServerThread.start()
        ltlmop_logger.info("Executor listening for XML-RPC calls on http://127.0.0.1:{} ...".format(listen_port))

        # Start the GUI if necessary
        if show_gui:
            # Create a subprocess
            ltlmop_logger.info("Starting GUI window...")
            p_gui = subprocess.Popen([sys.executable, "-u", "-m", "lib.simGUI", str(listen_port)])

            # Wait for GUI to fully load, to make sure that
            # to make sure all messages are redirected
            e.externalEventTargetRegistered.wait()

        if spec_file is not None:
            # Tell executor to load spec & aut
            #if aut_file is None:
            #    aut_file = spec_file.rpartition('.')[0] + ".aut"
            e.initialize(spec_file, aut_file, firstRun=True)

        # Start the executor's main loop in this thread
        e.run()

        # Clean up on exit
        ltlmop_logger.info("Waiting for XML-RPC server to shut down...")
        xmlrpc_server.shutdown()
        XMLRPCServerThread.join()
        ltlmop_logger.info("XML-RPC server shutdown complete.  Goodbye.")
    except:
        print "Exception !!!:"
        traceback.print_exc(file=sys.stdout)
        traceback.print_stack()
        pdb.post_mortem()


### Command-line argument parsing ###

if __name__ == "__main__":
    ### Check command-line arguments

    aut_file = None
    spec_file = None
    show_gui = True
    listen_port = None

    try:
        opts, args = getopt.getopt(sys.argv[1:], "hnp:a:s:", ["help", "no-gui", "xmlrpc-listen-port=", "aut-file=", "spec-file="])
    except getopt.GetoptError:
        ltlmop_logger.exception("Bad arguments")
        usage(sys.argv[0])
        sys.exit(2)

    for opt, arg in opts:
        if opt in ("-h", "--help"):
            usage(sys.argv[0])
            sys.exit()
        elif opt in ("-n", "--no-gui"):
            show_gui = False
        elif opt in ("-p", "--xmlrpc-listen-port"):
            try:
                listen_port = int(arg)
            except ValueError:
                ltlmop_logger.error("Invalid port '{}'".format(arg))
                sys.exit(2)
        elif opt in ("-a", "--aut-file"):
            aut_file = arg
        elif opt in ("-s", "--spec-file"):
            spec_file = arg

    execute_main(listen_port, spec_file, aut_file, show_gui)
