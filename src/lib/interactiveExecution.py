#!/usr/bin/python
#
# Interactive execution
#
import math
import os
import sys, code
import resource
import subprocess
import signal
import tempfile
import copy
import itertools
import numpy as np

# ==================================
# Settings
# ==================================
MAGNIFY = 64

# ==================================
# Entry point
# ==================================
if len(sys.argv)<2:
    print >>sys.stderr, "Error: Need BDD file"
    sys.exit(1)
bddinfile = sys.argv[1]
fileBasis = bddinfile[0:bddinfile.rfind(".bdd")]
   

# ==================================
# Prepare Slugs Call
# ==================================
print "Using BDD file: "+bddinfile
slugsLink = sys.argv[0][0:sys.argv[0].rfind("interactiveExecution.py")]+"../etc/slugs/src/slugs"
print "Using SLUGS at: "+slugsLink

# ==================================
# Main loop
# ==================================
def actionLoop():
    
    slugsinfile = fileBasis+".slugsin"
    
    # Open Slugs
    slugsProcess = subprocess.Popen(slugsLink+" --interactiveStrategy "+slugsinfile, shell=True, bufsize=1048000, stdin=subprocess.PIPE, stdout=subprocess.PIPE)

    # Get input APs
    slugsProcess.stdin.write("XPRINTINPUTS\n")
    slugsProcess.stdin.flush()
    slugsProcess.stdout.readline() # Skip the prompt
    lastLine = " "
    inputAPs = []
    while (lastLine!=""):
        lastLine = slugsProcess.stdout.readline().strip()
        if lastLine!="":
            inputAPs.append(lastLine)

    # Get output APs
    slugsProcess.stdin.write("XPRINTOUTPUTS\n")
    slugsProcess.stdin.flush()
    slugsProcess.stdout.readline() # Skip the prompt
    lastLine = " "
    outputAPs = []
    while (lastLine!=""):
        lastLine = slugsProcess.stdout.readline().strip()
        if lastLine!="":
            outputAPs.append(lastLine)

    # Get initial state
    slugsProcess.stdin.write("XGETINIT\n")
    slugsProcess.stdin.flush()
    slugsProcess.stdout.readline() # Skip the prompt
    currentState = slugsProcess.stdout.readline().strip()

    # run XMAKETRANS again to get a non-trivial initial state (why?)
    initInput = currentState[0:len(inputAPs)]
    slugsProcess.stdin.write("XMAKETRANS_INIT\n"+initInput)
    slugsProcess.stdin.flush()
    slugsProcess.stdout.readline() # Skip the prompt
    currentState = slugsProcess.stdout.readline().strip() 

    if currentState=="ERROR":
        raise 123

    while 1:
        loopNumber += 1

        # PICK THE NEXT VALUES HERE
        # nextInput = ...
       
        # Make the transition
        slugsProcess.stdin.write("XMAKETRANS\n"+nextInput)
        slugsProcess.stdin.flush()
        slugsProcess.stdout.readline() # Skip the prompt
        nextLine = slugsProcess.stdout.readline().strip()
        if not nextLine.startswith("ERROR"):
            currentState = nextLine
            # Print the state
            print >>sys.stderr, currentState
            # Tick
            clock.tick(10)
        else:
            break;
        
        # DO SOMETHING WITH CURRENT STATE
                        

# ==================================
# Call main program
# ==================================
actionLoop()
