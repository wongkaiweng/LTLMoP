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
import fsa, project
from copy import deepcopy
from SimpleXMLRPCServer import SimpleXMLRPCServer, SimpleXMLRPCRequestHandler
import xmlrpclib
import socket
import random
import math
import traceback
from resynthesis import ExecutorResynthesisExtensions
import globalConfig, logging

###### ENV VIOLATION CHECK ######
import copy
import specCompiler

lib_path = os.path.abspath('../src/LTLparser')
if lib_path not in sys.path:
    sys.path.append(lib_path)
import LTLcheck

#################################

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

class LTLMoPExecutor(ExecutorResynthesisExtensions, object):
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
        self.aut = None

        # Choose a timer func with maximum accuracy for given platform
        if sys.platform in ['win32', 'cygwin']:
            self.timer_func = time.clock
        else:
            self.timer_func = time.time

        self.externalEventTarget = None
        self.externalEventTargetRegistered = threading.Event()
        self.postEventLock = threading.Lock()
        self.runFSA = threading.Event()  # Start out paused
        self.alive = threading.Event()
        self.alive.set()
        
        ########## ENV Assumption Learning ######
        self.compiler = None                   
        self.LTLViolationCheck = None
        self.path_LTLfile = None                    # path of the .ltl file
        self.LTL2SpecLineNumber = None          # mapping from LTL to structed english
        #########################################

    def postEvent(self, eventType, eventData=None):
        """ Send a notice that an event occurred, if anyone wants it """
        
        with self.postEventLock:
            if self.externalEventTarget is None:
                return

            try:
                self.externalEventTarget.handleEvent(eventType, eventData)
            except socket.error as e:
                logging.warning("Could not send event to remote event target: %s", e)
                logging.warning("Forcefully unsubscribing target.")
                self.externalEventTarget = None

    def loadSpecFile(self, filename):
        # Update with this new project
        self.proj = project.Project()
        self.proj.loadProject(filename)

        # Tell GUI to load the spec file
        self.postEvent("SPEC", self.proj.getFilenamePrefix() + ".spec")

    def loadAutFile(self, filename):
        logging.info("Loading automaton...")

        aut = fsa.Automaton(self.proj)

        success = aut.loadFile(filename, self.proj.enabled_sensors, self.proj.enabled_actuators, self.proj.all_customs + self.proj.internal_props)

        return aut if success else None

    def _getCurrentRegionFromPose(self, rfi=None):
        # TODO: move this to regions.py
        if rfi is None:
            rfi = self.proj.rfi

        pose = self.proj.coordmap_lab2map(self.proj.h_instance['pose'].getPose())

        region = next((i for i, r in enumerate(rfi.regions) if r.name.lower() != "boundary" and \
                        r.objectContainsPoint(*pose)), None)
 
        if region is None:
            logging.warning("Pose of {} not inside any region!".format(pose))

        return region

    def shutdown(self):
        self.runFSA.clear()
        logging.info("QUITTING.")

        all_handler_types = ['init', 'pose', 'locomotionCommand', 'drive', 'motionControl', 'sensor', 'actuator']

        for htype in all_handler_types:
            logging.info("Terminating {} handler...".format(htype))
            if htype in self.proj.h_instance:
                if isinstance(self.proj.h_instance[htype], dict):
                    handlers = [v for k,v in self.proj.h_instance[htype].iteritems()]
                else:
                    handlers = [self.proj.h_instance[htype]]
            
                for h in handlers:
                    if hasattr(h, "_stop"):
                        logging.debug("Calling _stop() on {}".format(h.__class__.__name__))
                        h._stop()
                    else:
                        logging.debug("{} does not have _stop() function".format(h.__class__.__name__))
            else:
                logging.debug("{} handler not found in h_instance".format(htype))

        self.alive.clear()

    def pause(self):
        """ pause execution of the automaton """
        self.runFSA.clear()
        time.sleep(0.1) # Wait for FSA to stop
        self.postEvent("PAUSE")

    def resume(self):
        """ start/resume execution of the automaton """
        self.runFSA.set()

    def isRunning(self):
        """ return whether the automaton is currently executing """
        return self.runFSA.isSet()

    def getCurrentGoalNumber(self):
        """ Return the index of the goal currently being pursued (jx).
            If no automaton is loaded, return None. """
        if not self.aut:
            return None
        else:
            return self.aut.next_state.rank

    def registerExternalEventTarget(self, address):
        self.externalEventTarget = xmlrpclib.ServerProxy(address, allow_none=True)

        # Redirect all output to the log
        redir = RedirectText(self.externalEventTarget.handleEvent)

        sys.stdout = redir
        sys.stderr = redir

        self.externalEventTargetRegistered.set()

    def initialize(self, spec_file, aut_file, firstRun=True):
        """
        Prepare for execution, by loading and initializing all the relevant files (specification, map, handlers, aut)

        If `firstRun` is true, all handlers will be imported; otherwise, only the motion control handler will be reloaded.
        """

        # load project only first time; otherwise self.proj is modified in-place
        # TODO: make this less hacky
        if firstRun:
            self.loadSpecFile(spec_file)

            if self.proj.compile_options['decompose']:
                self.proj.rfiold = self.proj.rfi  # Save the undecomposed regions

        if self.proj.compile_options['decompose']:
            self.proj.rfi = self.proj.loadRegionFile(decomposed=True)

        if self.proj.currentConfig is None:
            logging.error("Can not simulate without a simulation configuration.")
            logging.error("Please create one by going to [Run] > [Configure Simulation...] in SpecEditor and then try again.")
            sys.exit(2)

        # HACK: give the proj a reference to executor
        self.proj.executor = self        
        
        # Import the relevant handlers
        if firstRun:
            logging.info("Importing handler functions...")
            self.proj.importHandlers()
        else:
            #print "Reloading motion control handler..."
            #self.proj.importHandlers(['motionControl'])
            pass

        # We are done initializing at this point if there is no aut file yet
        if aut_file is None:
            return

        # Load automaton file
        new_aut = self.loadAutFile(aut_file)

        if firstRun:
            ### Wait for the initial start command
            logging.info("Ready.  Press [Start] to begin...")
            self.runFSA.wait()

        ### Figure out where we should start from

        init_region = self._getCurrentRegionFromPose()
        if init_region is None:
            logging.error("Initial pose not inside any region!")
            sys.exit(-1)

        logging.info("Starting from initial region: " + self.proj.rfi.regions[init_region].name)

        ### Have the FSA find a valid initial state

        if firstRun or self.aut is None:
            # Figure out our initially true outputs
            init_outputs = []
            for prop in self.proj.currentConfig.initial_truths:
                if prop not in self.proj.enabled_sensors:
                    init_outputs.append(prop)

            init_state = new_aut.chooseInitialState(init_region, init_outputs)
        else:
            # Figure out our initially true outputs
            init_outputs = [k for k,v in self.aut.getCurrentState().outputs.iteritems() if int(v) == 1]
            self.postEvent("INFO",str(init_outputs)) 
            self.postEvent("INFO",str(self.aut.getCurrentState().outputs))    
            init_state = new_aut.chooseInitialState(init_region, init_outputs)#, goal=prev_z)

        if init_state is None:
            logging.error("No suitable initial state found; unable to execute. Quitting...")
            sys.exit(-1)
        else:
            logging.info("Starting from state %s." % init_state.name)

        self.aut = new_aut
        
        ######## ENV Assumption Learning ###########
        if firstRun:
            self.compiler = specCompiler.SpecCompiler(spec_file)
            self.compiler._decompose()  # WHAT DOES IT DO? DECOMPOSE REGIONS?
            self.spec, traceback, response = self.compiler._writeLTLFile(False)
            self.spec['EnvTrans'] = "\t[](FALSE) &\n"
            self.recreateLTLfile(self.proj)
            self.path_LTLfile =  os.path.join(self.proj.project_root,self.proj.getFilenamePrefix()+".ltl")  # path of ltl file to be passed to the function 
            self.LTLViolationCheck = LTLcheck.LTL_Check(self.path_LTLfile,self.compiler.LTL2SpecLineNumber,self.spec)
        #############################################

    def run(self):
        ### Get everything        print self.current_outputs moving
        # Rate limiting is approximately 20Hz
        avg_freq = 20
        last_gui_update_time = 0

        # FIXME: don't crash if no spec file is loaded initially
        while self.alive.isSet():
            # Idle if we're not running
            if not self.runFSA.isSet():
                try:
                    self.proj.h_instance['motionControl'].stop()
                except AttributeError:
                    self.proj.h_instance['drive'].setVelocity(0,0)
                
                ###### ENV VIOLATION CHECK ######
                # for generation of specification from DNF to CNF

                #print self.compiler._synthesize(False, True, True)[2]  # TRUE for realizable, FALSE for unrealizable
                self.postEvent("INFO", self.compiler._synthesize(False, True, True)[2])
                ################################
                    
                # wait for either the FSA to unpause or for termination
                while (not self.runFSA.wait(0.1)) and self.alive.isSet():
                    pass

            # Exit immediately if we're quitting
            if not self.alive.isSet():
                break    

            ################### is moved to initialize ######
            #init_state = self.aut.chooseInitialState(init_region, init_outputs)
            #print "init_region: " + str(init_region)   # by Catherine
            #print "init_outputs: " + str(init_outputs)  # by Catherine
            ############### see if necessary later ##### 

            self.prev_outputs = deepcopy(self.aut.current_outputs)
            self.prev_z = self.aut.current_state.rank

            tic = self.timer_func()
            self.aut.runIteration()

            ###### ENV VIOLATION CHECK ######
            #$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$##
            #$$$$$ DISABLE NOW SINCE THIS IS FOR LIVENESS GENERATION $$$$$$$$$$##
#            if initial_sensor_state == False:
#                self.LTLViolationCheck.sensor_state_len = len(self.aut.getCurrentState().inputs)
#                # store the name of the sensors
#                for key,value in self.aut.getSensorState().iteritems():
#                    self.LTLViolationCheck.sensors.append(key)
            #$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$##
            
            # Check for environment violation
            env_assumption_hold = self.LTLViolationCheck.checkViolation(self.aut.getCurrentState(),self.aut.getSensorState())

            # change the env_assumption_hold to int again (messed up by Python? )
            env_assumption_hold = int(env_assumption_hold)
            print env_assumption_hold
            # temporarily added to account for [](FALSE)
            #self.LTLViolationCheck.modify_LTL_file()
            
            # Modify the ltl file based on the enviornment change
            if env_assumption_hold == False:
            
                # Add the current state in init state of the LTL spec
                self.postEvent("INFO","Adding current state to init condition of LTL formulas")
                self._setSpecificationInitialConditionsToCurrentInDNF(self.proj)
                
                self.postEvent("INFO","Going to modify .ltl file to account for new env detected")
                self.spec['EnvTrans'] = self.LTLViolationCheck.modify_LTL_file()
                self.recreateLTLfile(self.proj)
                self.postEvent("INFO",str(self.spec['EnvTrans']) )
                
                realizable, realizableFS, output = self.compiler._synthesize()  # TRUE for realizable, FALSE for unrealizable
                self.postEvent("INFO", str(realizable)+ ", " + str(realizableFS) + ": " + str(output))
                
                self.postEvent("INFO", "\n\nINITIAL:modify_stage: " + str(self.LTLViolationCheck.modify_stage) + "-realizable: " + str(realizable))

                if not realizable:
                    while self.LTLViolationCheck.modify_stage < 3 and not realizable:
                        self.LTLViolationCheck.modify_stage += 1 

                        self.spec['EnvTrans'] = self.LTLViolationCheck.modify_LTL_file()
                        self.recreateLTLfile(self.proj)

                        realizable, realizableFS, output  = self.compiler._synthesize()  # TRUE for realizable, FALSE for unrealizable
                        self.postEvent("INFO", str(realizable)+ ", " + str(realizableFS) + ": " + str(output))


                self.postEvent("INFO", "STAGE: " + str(self.LTLViolationCheck.modify_stage))
                self.postEvent("INFO","FINAL:-realizable: " + str(realizable))
                # reload aut file if the new ltl is realizable        
                if realizable:
                    self.postEvent("INFO", "ViolationSolved:")
                    self.LTLViolationCheck.sameState = False
                    #######################
                    # Load automaton file #
                    #######################
                    self.postEvent("INFO","Reloading automaton...")
                    #FSA = fsa.Automaton(proj)
##                    self.runFSA.clear() #runFSA = False

##                    success = self.aut.loadFile(aut_file, self.proj.enabled_sensors, self.proj.enabled_actuators, self.proj.all_customs)
##                    if not success: return
##                    #cur_region_no = 0
##                    cur_outputs = [] #format: ['act1','act2']
##                    for key, value in self.LTLViolationCheck.current_state.outputs.iteritems():
##                        if key.find("bit") == -1 and (int(value) == 1):
##                            cur_outputs.append(key)
##                        self.postEvent("INFO", str(key) + ": " +str(value))
##                    cur_region_no = self.aut.regionFromState(self.LTLViolationCheck.current_state)
##                    
##                    self.postEvent("INFO", "cur_region_no:" + str(cur_region_no))
##                    #init_state = self.aut.chooseInitialState(init_region, init_outputs)
##                    init_state = self.aut.chooseInitialState(cur_region_no, cur_outputs)
##                    #print "cur_outputs: " + str(cur_outputs)  # by Catherine
##                    
##                    if init_state is None:
##                        self.postEvent("INFO",  "No suitable initial state found; unable to execute. Quitting...")
##                        sys.exit(-1)
##                    else:
##                        self.postEvent("INFO",  "Starting from state %s." % init_state.name)
##                    
##                    self.runFSA.set() #runFSA = True
                    spec_file = self.proj.getFilenamePrefix() + ".spec"
                    aut_file = self.proj.getFilenamePrefix() + ".aut"
                    self.postEvent("INFO","initilazing...")
                    self.initialize(spec_file, aut_file, firstRun=False)
                        
                else:
                    self.postEvent("INFO",  "-----------------------------------------------")
                    # still missing removing liveness guarantees
                    self.postEvent("INFO",  "Trying to remove system liveness guarantees...")
                    self.LTLViolationCheck.remove_liveness_guarantees()
                    realizable = self.compiler._synthesize()[0]  # TRUE for realizable, FALSE for unrealizable
                    self.postEvent("INFO", str(realizable))    
                    
                    if realizable:
                        self.postEvent("INFO",  "Synthesizable without system liveness guarantees. Please put in some environment liveness assumptions."  )      
                        #time.sleep(5)
                          
                    else:
                        self.postEvent("INFO",  "Unknown error: please check your system safety guarantees" )
                        sys.exit()
                        
                         
                    self.LTLViolationCheck.append_liveness_guarantees()
                    self.postEvent("INFO",  "Trying to append liveness assumptions")
                    self.LTLViolationCheck.generate_env_livenss_assumptions(True)
                    realizable = self.compiler._synthesize()[0]  # TRUE for realizable, FALSE for unrealizable
                    self.postEvent("INFO", str(realizable))
                    
                    while not realizable and self.LTLViolationCheck.liveness_generation_count < (2*self.LTLViolationCheck.sensor_state_len + + 2**self.LTLViolationCheck.sensor_state_len):
                        self.LTLViolationCheck.generate_env_livenss_assumptions()
                        realizable = self.compiler._synthesize()[0]  # TRUE for realizable, FALSE for unrealizable
                        self.postEvent("INFO", str(realizable))
                        
                    if not realizable:
                        print "Still not realizable. Now we will exit the execution"
                        print "----------------------------------------------"
                        sys.exit()
                    else:
                        self.LTLViolationCheck.modify_stage  = 1 # reset the stage to 1
                
            
            else:    
                #if prev_cur_state != self.aut.current_state or prev_sensor_state != self.aut.sensor_state:
                    #print "The SENSOR state has been changed."
                    #print "Before:" + self.LTLViolationCheck.env_safety_assumptions_stage["3"]
                self.LTLViolationCheck.append_state_to_LTL(self.aut.getCurrentState(),self.aut.getSensorState())
                #self.LTLViolationCheck.modify_LTL_file()
                if env_assumption_hold == False:
                    print >>sys.__stdout__,"Value should be True: " + str(env_assumption_hold)
            #    print >>sys.__stdout__,"env_assumption_hold equals False: " + str(env_assumption_hold == False) +" should say False."     
                #if prev_cur_state != self.aut.current_state or prev_sensor_state != self.aut.sensor_state:
                #    print "After: " + self.LTLViolationCheck.env_safety_assumptions_stage["3"]
                    
            #prev_cur_state = self.aut.getCurrentState()
            #prev_sensor_state = self.aut.getSensorState() 
            #################################
            
            toc = self.timer_func()

            # Rate limiting of execution and GUI update
            while (toc - tic) < 0.05:
                time.sleep(0.005)
                toc = self.timer_func()

            # Update GUI
            # If rate limiting is disabled in the future add in rate limiting here for the GUI:
            # if show_gui and (timer_func() - last_gui_update_time > 0.05)
            avg_freq = 0.9 * avg_freq + 0.1 * 1 / (toc - tic) # IIR filter
            self.postEvent("FREQ", int(math.ceil(avg_freq)))
            pose = self.proj.h_instance['pose'].getPose(cached=True)[0:2]
            self.postEvent("POSE", tuple(map(int, self.proj.coordmap_lab2map(pose))))

            last_gui_update_time = self.timer_func()

        logging.debug("execute.py quitting...")

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
    logging.info("Hello. Let's do this!")

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
    
    # Create the execution context object
    e = LTLMoPExecutor()

    # Register functions with the XML-RPC server
    xmlrpc_server.register_instance(e)

    # Kick off the XML-RPC server thread    
    XMLRPCServerThread = threading.Thread(target=xmlrpc_server.serve_forever)
    XMLRPCServerThread.daemon = True
    XMLRPCServerThread.start()
    logging.info("Executor listening for XML-RPC calls on http://127.0.0.1:{} ...".format(listen_port))

    # Start the GUI if necessary
    if show_gui:
        # Create a subprocess
        logging.info("Starting GUI window...")
        p_gui = subprocess.Popen(["python", "-u", os.path.join(project.get_ltlmop_root(), "lib", "simGUI.py"), str(listen_port)])

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
    logging.info("Waiting for XML-RPC server to shut down...")
    xmlrpc_server.shutdown()
    XMLRPCServerThread.join()


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
        logging.exception("Bad arguments") 
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
                logging.error("Invalid port '{}'".format(arg))
                sys.exit(2)
        elif opt in ("-a", "--aut-file"):
            aut_file = arg
        elif opt in ("-s", "--spec-file"):
            spec_file = arg

    execute_main(listen_port, spec_file, aut_file, show_gui)
