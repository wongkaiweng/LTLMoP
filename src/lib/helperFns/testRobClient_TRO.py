import os
import sys
p = os.path.abspath(__file__)
print os.path.join(os.path.dirname(p),"../")
sys.path.append(os.path.join(os.path.dirname(p),"../"))

import project
import negotiationMonitor.robotClient
import handlerSubsystem
import specCompiler
import strategy
import logging
import time

logging.getLogger().setLevel(logging.DEBUG)
#logging.basicConfig(level=logging.DEBUG)

def loadRobotClient(specpath, otherRobotName):
    """
    specpath: path to specification
    """
    executor = None
    proj = project.Project()
    proj.loadProject(specpath)
    hsub = handlerSubsystem.HandlerSubsystem(executor, proj.project_root)
    hsub.setExecutingConfig(proj.current_config)
    proj.rfi = proj.loadRegionFile(decomposed=True)
    robClient = negotiationMonitor.robotClient.RobotClient(hsub,proj)

    # loadStrategy
    region_domain = [strategy.Domain("region", proj.rfi.regions, strategy.Domain.B0_IS_MSB)]
    regionCompleted_domain = [strategy.Domain("regionCompleted", proj.rfi.regions, strategy.Domain.B0_IS_MSB)]
    enabled_sensors = [x for x in proj.enabled_sensors if not x.endswith('_rc') or x.startswith(otherRobotName)]
    strat = strategy.createStrategyFromFile(specpath.replace('.spec','.aut'),
                                                enabled_sensors + regionCompleted_domain,
                                                proj.enabled_actuators + proj.all_customs  + proj.internal_props + region_domain)
    return robClient, strat

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


if __name__ == '__main__':
    #ltlSpecA = '../examples/FStwo_robot_negotiation/alice/alice.spec'
    #ltlSpecB = '../examples/FStwo_robot_negotiation/bob/bob.spec'
    ## specify initial state
    #aliceStateDict = {'bob_r1':1, 'bob_r2':0, 'bob_r3':0, 'bob_r4':0, 'bob_r5':0, 'bob_r3_rc':0, 'bob_r5_rc':0, 'bob_r2_rc':0, 'bob_r4_rc':0, 'bob_r1_rc':1, 'regionCompleted_b0':0, 'regionCompleted_b1':0, 'regionCompleted_b2':1, 'region_b0':0, 'region_b1':0, 'region_b2':1}
    #bobStateDict  = {'alice_r1':0, 'alice_r2':0, 'alice_r3':1, 'alice_r4':0, 'alice_r5':0, 'alice_r3_rc':1, 'alice_r5_rc':0, 'alice_r2_rc':0, 'alice_r4_rc':0, 'alice_r1_rc':0, 'regionCompleted_b0':1, 'regionCompleted_b1':0, 'regionCompleted_b2':0, 'region_b0':1, 'region_b1':0, 'region_b2':0}
    ltlSpecA = '../../examples/FStwo_robot_negotiation/kitchen/chef/chef.spec'
    ltlSpecB = '../../examples/FStwo_robot_negotiation/kitchen/kitchenAssistant/assistant.spec'
    ## specify initial state
    #aliceStateDict = {ingredientArrived:0, assistant_passIngredient:0, assistant_ActpassIngredient:0, cooking_ac:0, sbit0:0, sbit1:1, sbit2:1, cooking:0, receivedIngredient:0, bit0:0, bit1:1, bit2:1}
    #bobStateDict  = {ingredientArrived:0, chef_receivedIngredient:0, chef_cooking:0, deliveryAgent_road:0, deliveryAgent_storage:0, deliveryAgent_orderDelivery:0, deliveryAgent_company:0, passIngredient_ac:0, openDoor_ac:0, sbit0:0, sbit1:0, sbit2:1, passIngredient:0, openDoor:0, bit0:0, bit1:0, bit2:1}

    # load spec
    aliceSpec = loadSpecFile(ltlSpecA)
    bobSpec = loadSpecFile(ltlSpecB)

    # load client and strategy
    aliceClient, aliceStrat = loadRobotClient(ltlSpecA,'assistant')
    bobClient, bobStrat = loadRobotClient(ltlSpecB,'chef')

    ## load initial state
    #aliceState = aliceStrat.searchForOneState(aliceStateDict)
    #bobState = bobStrat.searchForOneState(bobStateDict)

    # send spec
    for specType, specStr in aliceSpec.iteritems():
        if specType not in ['SysInit','SysTrans','SysGoals','EnvInit','EnvTrans','EnvGoals']:
            continue
        aliceClient.sendSpec(specType, specStr, fastslow=True, include_heading=False)

    for specType, specStr in bobSpec.iteritems():
        if specType not in ['SysInit','SysTrans','SysGoals','EnvInit','EnvTrans','EnvGoals']:
            continue
        bobClient.sendSpec(specType, specStr, fastslow=True, include_heading=False)


    print aliceClient.requestSpec('EnvTrans')
    print aliceClient.requestSpec('SysGoals')
    print bobClient.requestSpec('EnvTrans')
    print bobClient.requestSpec('SysGoals')

    ## send prop
    #aliceClient.sendProp('env',aliceState.getInputs(expand_domains = True))
    #aliceClient.sendProp('sys',aliceState.getOutputs(expand_domains = True))
    #bobClient.sendProp('env',bobState.getInputs(expand_domains = True))
    #bobClient.sendProp('sys',bobState.getOutputs(expand_domains = True))

    ## send cooridination status
    #aliceClient.setCoordinationStatus(True)
    #bobClient.setCoordinationStatus(True)

    #time.sleep(5)
    ## update next inputs and request outputs
    #print 'alice:' + str(aliceClient.getOutputs(aliceState.getInputs(expand_domains = True)))
    #print 'bob:' + str(bobClient.getOutputs(bobState.getInputs(expand_domains = True)))
    #print 'alice:' + str(aliceClient.getOutputs(aliceState.getInputs(expand_domains = True)))

