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

import multiprocessing #parallel processes

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
    patchExecutor = centralCoordinator.decentralizedPatchingExecutor.PatchingExecutor(hsub, proj)

    return patchExecutor

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

processes = []
def closeConnection(signal, frame):
    for process in processes:
        process.join()
    sys.exit(0)

ltlSpecA = '../../examples/FStwo_robot_negotiation/alice/alice_no_heading.spec'
ltlSpecB = '../../examples/FStwo_robot_negotiation/bob/bob_no_heading.spec'
ltlSpecC = '../../examples/patching/threeRobots/charlie/patching_charlie.spec'


#aliceProcess = multiprocessing.Process(name='aliceThread', target=initDPatching, args=(ltlSpecA,))
#aliceProcess.daemon = True
#aliceProcess.start()
#processes.append(aliceProcess)

#bobProcess = multiprocessing.Process(name='bobThread', target=initDPatching, args=(ltlSpecB,))
#bobProcess. daemon = True
#bobProcess.start()
#processes.append(bobProcess)

alicePatching = initDPatching(ltlSpecA)
bobPatching = initDPatching(ltlSpecB)
charliePatching = initDPatching(ltlSpecC)

# load spec
aliceSpec = loadSpecFile(ltlSpecA)
bobSpec = loadSpecFile(ltlSpecB)
charlieSpec = loadSpecFile(ltlSpecC)

signal.signal(signal.SIGINT, closeConnection) # terminate object if keyboard interrupt
while True:
    alicePatching.checkData()
    bobPatching.checkData()
    charliePatching.checkData()

    # catch error if we sampled the robot itself
    a_csock = random.choice(alicePatching.clients.values())
    if a_csock != alicePatching.serv:
        alicePatching.updateRobotRegion(a_csock, random.choice(alicePatching.regions))
        # send spec
        for specType, specStr in aliceSpec.iteritems():
            if specType not in ['SysInit', 'SysTrans', 'SysGoals', 'EnvInit', 'EnvTrans', 'EnvGoals']:
                continue
            alicePatching.sendSpec(a_csock, specType, specStr, fastslow=True, include_heading=True)


    b_sock = random.choice(bobPatching.clients.values())
    if b_sock != bobPatching.serv:
        bobPatching.updateRobotRegion(b_sock, random.choice(bobPatching.regions))
        # send spec
        for specType, specStr in bobSpec.iteritems():
            if specType not in ['SysInit', 'SysTrans', 'SysGoals', 'EnvInit', 'EnvTrans', 'EnvGoals']:
                continue
            bobPatching.sendSpec(b_sock, specType, specStr, fastslow=True, include_heading=True)

    c_sock = random.choice(charliePatching.clients.values())
    if c_sock != charliePatching.serv:
        charliePatching.updateRobotRegion(c_sock, random.choice(charliePatching.regions))
        # send spec
        for specType, specStr in charlieSpec.iteritems():
            if specType not in ['SysInit', 'SysTrans', 'SysGoals', 'EnvInit', 'EnvTrans', 'EnvGoals']:
                continue
            charliePatching.sendSpec(c_sock, specType, specStr, fastslow=True, include_heading=True)

    time.sleep(1)




