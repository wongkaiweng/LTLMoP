#!/usr/bin/env python
import sys, os
import time
import random
p = os.path.abspath(__file__)
print os.path.join(os.path.dirname(p), "../")
sys.path.append(os.path.join(os.path.dirname(p), "../"))
import signal
import logging

import centralCoordinator.decentralizedPatchingExecutor
import project
import handlerSubsystem
import specCompiler
import strategy
import project


import multiprocessing #parallel processes
import threading

import pdb

sys.setrecursionlimit(10**6)


def initDPatching(specpath):
    """
    specpath: path to specification
    """
    executor = None
    proj = project.Project()
    proj.loadProject(specpath)
    hsub = handlerSubsystem.HandlerSubsystem(executor, proj.project_root)
    hsub.setExecutingConfig(proj.current_config)
    proj.rfi = proj.loadRegionFile(decomposed=True)
    patchExecutor = centralCoordinator.decentralizedPatchingExecutor.PatchingExecutor(hsub, proj, testDPatchingMode=True)

    # loadStrategy
    region_domain = [strategy.Domain("region", proj.rfi.regions, strategy.Domain.B0_IS_MSB)]
    regionCompleted_domain = [strategy.Domain("regionCompleted", proj.rfi.regions, strategy.Domain.B0_IS_MSB)]
    enabled_sensors = [x for x in proj.enabled_sensors if not x.endswith('_rc')]
    strat = strategy.createStrategyFromFile(specpath.replace('.spec','.aut'),
                                                enabled_sensors + regionCompleted_domain,
                                                proj.enabled_actuators + proj.all_customs  + proj.internal_props + region_domain)
    return patchExecutor, strat

def loadSpecFile(spec_file):
    """
    spec_file: path to specification
    """
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

def sendSpec(csock, patchObject, specDict, stateObject, ltlSpecPath):
    """
    This function handles all the sending to other robot given csock
    """
    # send spec
    for specType, specStr in specDict.iteritems():
        if specType not in ['SysInit', 'SysTrans', 'SysGoals', 'EnvInit', 'EnvTrans', 'EnvGoals']:
            continue

        if specType == 'SysGoals':
            patchObject.sendSpec(csock, 'SysGoalsOld', specStr, fastslow=True, include_heading=True)
        patchObject.sendSpec(csock, specType, specStr, fastslow=True, include_heading=True)

    with open(ltlSpecPath.replace('.spec','.autWinPos'), 'r') as f:
        winPosSpec = f.read()
    f.closed
    patchObject.sendSpec(csock, 'WinPos', '('+winPosSpec+')', fastslow=True, include_heading=True)
    patchObject.sendProp(csock, 'env', stateObject.getInputs(expand_domains = True))

    sysProps = stateObject.getOutputs(expand_domains=True)
    sysProps = {sProp:False if 'region_b' in sProp else sValue for sProp, sValue in sysProps.iteritems()}

    for eProp, eValue in stateObject.getInputs(expand_domains=True).iteritems():
        if 'regionCompleted_b' in eProp:
            sysProps[eProp.replace('regionCompleted_b', 'region_b')] = eValue
    logging.warning('sysProps:' + str(sysProps))

    patchObject.sendProp(csock, 'sys', sysProps)

def sendSpecToMySelf(robotName, patchObject, specDict, stateObject, ltlSpecPath):
    """
    This function handles all the sending to myself
    """
    # send spec
    for specType, specStr in specDict.iteritems():
        if specType not in ['SysInit', 'SysTrans', 'SysGoals', 'EnvInit', 'EnvTrans', 'EnvGoals']:
            continue

        if specType == 'SysGoals':
            patchObject.sysGoalsOld[robotName] = patchObject.sendSpecHelper('SysGoalsOld', specStr, fastslow=True, include_heading=True)
        patchObject.spec[specType][robotName] = patchObject.sendSpecHelper(specType, specStr, fastslow=True, include_heading=True)

    with open(ltlSpecPath.replace('.spec','.autWinPos'), 'r') as f:
        winPosSpec = f.read()
    f.closed

    patchObject.winPos[robotName] = patchObject.sendSpecHelper('WinPos', '('+winPosSpec+')', fastslow=True, include_heading=True)
    patchObject.envPropList[robotName] = patchObject.sendPropHelper('env', stateObject.getInputs(expand_domains = True))
    patchObject.sysPropList[robotName] = patchObject.sendPropHelper('sys', stateObject.getOutputs(expand_domains = True))

    patchObject.propMappingNewToOld[robotName] = {}
    patchObject.propMappingOldToNew[robotName] = {}
    for prop in patchObject.envPropList[robotName].keys() + patchObject.sysPropList[robotName].keys():
        patchObject.propMappingNewToOld[robotName].update({prop:prop})
        patchObject.propMappingOldToNew[robotName].update({prop:prop})


processes = []
def closeConnection(signal, frame):
    for process in processes:
        process.join()
    sys.exit(0)

ltlSpecA = '../../examples/patching/threeRobots_small/alice/alice.spec'
ltlSpecB = '../../examples/patching/threeRobots_small/bob/bob.spec'
ltlSpecC = '../../examples/patching/threeRobots_small/charlie/charlie.spec'
# specify initial state
aliceStateDict = {'bob_M1':0, 'charlie_M1':1, 'bob_R2':1, 'charlie_R2':0, 'bob_R1':0, 'charlie_R1':0, 'bob_R3':0, 'charlie_R3':0, 'bob_ML':0, 'charlie_ML':0, 'bob_MR':0, 'charlie_MR':0,\
                   'bob_L3':0, 'charlie_L3':0, 'bob_L1':0, 'charlie_L1':0, 'bob_L2':0, 'charlie_L2':0, 'regionCompleted_b0':0, 'regionCompleted_b1':1, 'regionCompleted_b2':1,\
                    'regionCompleted_b3':1, 'region_b0':0, 'region_b1':1, 'region_b2':1, 'region_b3':1}
bobStateDict  = {'alice_M1':0, 'charlie_M1':1, 'alice_R2':0, 'charlie_R2':0, 'alice_R3':0, 'charlie_R3':0, 'alice_R1':0, 'charlie_R1':0, 'alice_ML':0, 'charlie_ML':0,\
                 'alice_MR':0, 'charlie_MR':0, 'alice_L3':0, 'charlie_L3':0, 'alice_L1':0, 'charlie_L1':0, 'alice_L2':1, 'charlie_L2':0, 'regionCompleted_b0':0, 'regionCompleted_b1':0,\
                  'regionCompleted_b2':0, 'regionCompleted_b3':0, 'region_b0':0, 'region_b1':0, 'region_b2':0, 'region_b3':0}
charlieStateDict  = {'alice_M1':0, 'bob_M1':0, 'alice_R2':0, 'bob_R2':1, 'alice_R3':0, 'bob_R3':0, 'alice_R1':0, 'bob_R1':0, 'alice_ML':0, 'bob_ML':0, 'alice_MR':0,\
                     'bob_MR':0, 'alice_L3':0, 'bob_L3':0, 'alice_L2':1, 'bob_L2':0, 'alice_L1':0, 'bob_L1':0, 'requestL3':0, 'requestR1':0, 'regionCompleted_b0':0,\
                      'regionCompleted_b1':0, 'regionCompleted_b2':1, 'regionCompleted_b3':1, 'region_b0':0, 'region_b1':0, 'region_b2':1, 'region_b3':1}


ltlSpecA = '../../examples/patching/maintainDistance_patching/alice_noCoordination/alice_no_heading.spec'
ltlSpecB = '../../examples/patching/maintainDistance_patching/bob_noCoordination/bob_no_heading.spec'
# specify initial state
aliceStateDict = {'bob_r1':0, 'bob_r3':0, 'bob_r4':1, 'bob_r5':0, 'bob_r6':0, 'bob_r2':0, 'regionCompleted_b0':0, 'regionCompleted_b1':1,\
                'regionCompleted_b2':1, 'region_b0':0, 'region_b1':1, 'region_b2':1}
bobStateDict  = {'alice_r1':1, 'alice_r2':0, 'alice_r3':0, 'alice_r4':0, 'alice_r5':0, 'alice_r6':0, 'regionCompleted_b0':0, 'regionCompleted_b1':0, \
                'regionCompleted_b2':1, 'region_b0':0, 'region_b1':0, 'region_b2':1}

alicePatching, aliceStrategy = initDPatching(ltlSpecA)
bobPatching, bobStrategy = initDPatching(ltlSpecB)
charliePatching, charlieStrategy = initDPatching(ltlSpecC)

def addProcess(threadName, functionName):
    processObject = threading.Thread(name=threadName, target=functionName)
    processObject.daemon = True
    processObject.start()
    processes.append(processObject)

addProcess('aliceThread', alicePatching.runCheckData)
addProcess('bobThread', bobPatching.runCheckData)
addProcess('charlieThread', charliePatching.runCheckData)

# load spec
aliceSpec = loadSpecFile(ltlSpecA)
bobSpec = loadSpecFile(ltlSpecB)
charlieSpec = loadSpecFile(ltlSpecC)


# load initial state
aliceState = aliceStrategy.searchForOneState(aliceStateDict)
bobState = bobStrategy.searchForOneState(bobStateDict)
charlieState = charlieStrategy.searchForOneState(charlieStateDict)

signal.signal(signal.SIGINT, closeConnection) # terminate object if keyboard interrupt
countThres = 2
count = 0
threeRobotsMode = False

while True:
    #alicePatching.checkData()
    #bobPatching.checkData()
    #charliePatching.checkData()

    if not count < countThres and not threeRobotsMode:
        ###### ALICE #####
        # update region info
        # catch error if we sampled the robot itself
        aliceRegion = random.choice(alicePatching.regions)
        for a_csock in alicePatching.clients.values():
            if a_csock != alicePatching.serv:
                alicePatching.updateRobotRegion(a_csock, aliceRegion)

        # sendToMyself and other robot
        sendSpecToMySelf('alice', alicePatching, aliceSpec, aliceState, ltlSpecA)
        sendSpec(alicePatching.clients['bob'], alicePatching, aliceSpec, aliceState,ltlSpecA)

        # set coorindatingRobots
        alicePatching.sendCoordinationRequest(alicePatching.clients['bob'], True)
        alicePatching.setCoordinationRequestSent(['bob'])
        #alicePatching.coordinatingRobots = ['alice','bob']


        ###### BOB #####
        # update region info
        #b_sock = random.choice(bobPatching.clients.values())
        bobRegion = random.choice(bobPatching.regions)
        for b_sock in bobPatching.clients.values():
            if b_sock != bobPatching.serv:
                bobPatching.updateRobotRegion(b_sock, bobRegion)

        # sendToMyself and other robot
        sendSpecToMySelf('bob', bobPatching, bobSpec, bobState, ltlSpecB)
        sendSpec(bobPatching.clients['alice'], bobPatching, bobSpec, bobState, ltlSpecB)

        # set coorindatingRobots
        bobPatching.sendCoordinationRequest(bobPatching.clients['alice'], True)
        bobPatching.setCoordinationRequestSent(['alice'])
        #bobPatching.coordinatingRobots = ['alice','bob']


        ###### CHARLIE #####
        # charlieRegion = random.choice(bobPatching.regions)
        # #c_sock = random.choice(charliePatching.clients.values())
        # for c_sock in charliePatching.clients.values():
        #     if c_sock != charliePatching.serv:
        #         charliePatching.updateRobotRegion(c_sock, charlieRegion)

    #two robot mode
    if not threeRobotsMode and alicePatching.prepareForCentralizedExecution():
        threeRobotsMode = True
        logging.warning('set threeRobotsMode to True!')
        #bobPatching.prepareForCentralizedExecution()

    # request the other robot to join
    if threeRobotsMode:
        sendSpec(charliePatching.clients['alice'], charliePatching, charlieSpec, charlieState, ltlSpecC)

        # set coorindatingRobots
        charliePatching.sendCoordinationRequest(charliePatching.clients['alice'], True)
        #charliePatching.coordinatingRobots = ['alice','bob']

        alicePatching.prepareForCentralizedExecution()

    count += 1


    time.sleep(1)

pdb.pm()




