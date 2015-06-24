# TODO: put into the directory of LTLMoP/src/lib to use
import sys
import os
p = os.path.abspath(__file__)
sys.path.append(os.path.join(p,"src","lib"))

import LTLParser.LTLcheck
import fsa

def checkSpecWithInputsOutputs(spec, path, input_propositions, output_propositions, current_dict, sensor_dict):
    """
    spec: contains EnvTrans 
    path: path to ltl file
    input_propositions: list
    output_propositions: list
    current_dict: dictionary
    sensor_dict: dict
    """
    new_strategy = fsa.FSAStrategy()
    new_strategy.configurePropositions(input_propositions, output_propositions)

    # set current state
    current_state = new_strategy.states.addNewState()
    for prop_name, value in current_dict.iteritems():
        current_state.setPropValue(prop_name, value)

    # set next state
    sensor_strategy = new_strategy.states.addNewState()
    for prop_name, value in sensor_dict.iteritems():
        sensor_strategy.setPropValue(prop_name, value)

    LTLViolationCheck = LTLParser.LTLcheck.LTL_Check(path,{},spec)
    print LTLViolationCheck.checkViolation(current_state, sensor_strategy, LTLMoP = False)

    print LTLViolationCheck.violated_specStr

input_propositions = ['bob_policeStation2','bob_groceryStore', 'bob_bridge', 'bob_tunnel', 'bob_postOffice', 'bob_park', 'bob_policeStation1', 'bob_square','sbit0', 'sbit1', 'sbit2']
output_propositions = ['bit0', 'bit1', 'bit2']
current_dict = {'bob_policeStation2':False, 'bob_groceryStore':True, 'bob_bridge':False, 'bob_tunnel':False, 'bob_postOffice':False, 'bob_park':False, 'bob_policeStation1':False, 'bob_square':False, 'sbit0':True, 'sbit1':True, 'sbit2':True, 'bit0':False, 'bit1':False, 'bit2':False}
sensor_dict = {'bob_policeStation2':False, 'bob_groceryStore':True, 'bob_bridge':False, 'bob_tunnel':False, 'bob_postOffice':False, 'bob_park':False, 'bob_policeStation1':False, 'bob_square':False,'sbit0':False, 'sbit1':False, 'sbit2':False, 'bit0':False, 'bit1':False, 'bit2':False}

spec = {}
spec['EnvTrans'] = '			 []((( ((e.sbit0 & e.sbit1 & e.sbit2)) ) ) -> (   !  next(e.bob_square)) ) & \
			 []((( ((!e.sbit0 & !e.sbit1 & !e.sbit2)) ) ) -> (   !  next(e.bob_groceryStore)) ) & \
			 []((( ((!s.bit0 & !s.bit1 & !s.bit2)) )  |  ( ((!e.sbit0 & !e.sbit1 & !e.sbit2)) ) ) -> (   !  next(e.bob_square)) ) & \
			 []((( ((s.bit0 & s.bit1 & !s.bit2)) )  |  ( ((e.sbit0 & e.sbit1 & !e.sbit2)) ) ) -> (   !  next(e.bob_groceryStore)) ) & \
			 []( (e.bob_square) -> ( (next(e.bob_square))\
									| (next(e.bob_policeStation2)) \
									| (next(e.bob_bridge)) \
									| (next(e.bob_groceryStore)) \
									| (next(e.bob_tunnel))  ) )  & \
			 []( (e.bob_park) -> ( (next(e.bob_park))\
									| (next(e.bob_postOffice)) \
									| (next(e.bob_bridge)) \
									| (next(e.bob_tunnel)) \
									| (next(e.bob_policeStation1))  ) )  & \
			 []( (e.bob_postOffice) -> ( (next(e.bob_postOffice))\
									| (next(e.bob_park))  ) )  & \
			 []( (e.bob_policeStation2) -> ( (next(e.bob_policeStation2))\
									| (next(e.bob_square))  ) )  & \
			 []( (e.bob_bridge) -> ( (next(e.bob_bridge))\
									| (next(e.bob_square)) \
									| (next(e.bob_park))  ) )  & \
			 []( (e.bob_groceryStore) -> ( (next(e.bob_groceryStore))\
									| (next(e.bob_square))  ) )  & \
			 []( (e.bob_tunnel) -> ( (next(e.bob_tunnel))\
									| (next(e.bob_square)) \
									| (next(e.bob_park))  ) )  & \
			 []( (e.bob_policeStation1) -> ( (next(e.bob_policeStation1))\
									| (next(e.bob_park))  ) )  & \
[]((next(e.bob_square) & !next(e.bob_park) & !next(e.bob_postOffice) & !next(e.bob_policeStation2) & !next(e.bob_bridge) & !next(e.bob_groceryStore) & !next(e.bob_tunnel) & !next(e.bob_policeStation1)) |\
 (!next(e.bob_square) & next(e.bob_park) & !next(e.bob_postOffice) & !next(e.bob_policeStation2) & !next(e.bob_bridge) & !next(e.bob_groceryStore) & !next(e.bob_tunnel) & !next(e.bob_policeStation1)) |\
 (!next(e.bob_square) & !next(e.bob_park) & next(e.bob_postOffice) & !next(e.bob_policeStation2) & !next(e.bob_bridge) & !next(e.bob_groceryStore) & !next(e.bob_tunnel) & !next(e.bob_policeStation1)) |\
 (!next(e.bob_square) & !next(e.bob_park) & !next(e.bob_postOffice) & next(e.bob_policeStation2) & !next(e.bob_bridge) & !next(e.bob_groceryStore) & !next(e.bob_tunnel) & !next(e.bob_policeStation1)) |\
 (!next(e.bob_square) & !next(e.bob_park) & !next(e.bob_postOffice) & !next(e.bob_policeStation2) & next(e.bob_bridge) & !next(e.bob_groceryStore) & !next(e.bob_tunnel) & !next(e.bob_policeStation1)) |\
 (!next(e.bob_square) & !next(e.bob_park) & !next(e.bob_postOffice) & !next(e.bob_policeStation2) & !next(e.bob_bridge) & next(e.bob_groceryStore) & !next(e.bob_tunnel) & !next(e.bob_policeStation1)) |\
 (!next(e.bob_square) & !next(e.bob_park) & !next(e.bob_postOffice) & !next(e.bob_policeStation2) & !next(e.bob_bridge) & !next(e.bob_groceryStore) & next(e.bob_tunnel) & !next(e.bob_policeStation1)) |\
 (!next(e.bob_square) & !next(e.bob_park) & !next(e.bob_postOffice) & !next(e.bob_policeStation2) & !next(e.bob_bridge) & !next(e.bob_groceryStore) & !next(e.bob_tunnel) & next(e.bob_policeStation1)))\
&\
			 []( ((!e.sbit0 & !e.sbit1 & !e.sbit2) & (!s.bit0 & !s.bit1 & !s.bit2)) -> ((!next(e.sbit0)&!next(e.sbit1)&!next(e.sbit2))) )  & \
			 []( ((!e.sbit0 & !e.sbit1 & e.sbit2) & (!s.bit0 & !s.bit1 & s.bit2)) -> ((!next(e.sbit0)&!next(e.sbit1)&next(e.sbit2))) )  & \
			 []( ((!e.sbit0 & e.sbit1 & !e.sbit2) & (!s.bit0 & s.bit1 & !s.bit2)) -> ((!next(e.sbit0)&next(e.sbit1)&!next(e.sbit2))) )  & \
			 []( ((!e.sbit0 & e.sbit1 & e.sbit2) & (!s.bit0 & s.bit1 & s.bit2)) -> ((!next(e.sbit0)&next(e.sbit1)&next(e.sbit2))) )  & \
			 []( ((e.sbit0 & !e.sbit1 & !e.sbit2) & (s.bit0 & !s.bit1 & !s.bit2)) -> ((next(e.sbit0)&!next(e.sbit1)&!next(e.sbit2))) )  & \
			 []( ((e.sbit0 & !e.sbit1 & e.sbit2) & (s.bit0 & !s.bit1 & s.bit2)) -> ((next(e.sbit0)&!next(e.sbit1)&next(e.sbit2))) )  & \
			 []( ((e.sbit0 & e.sbit1 & !e.sbit2) & (s.bit0 & s.bit1 & !s.bit2)) -> ((next(e.sbit0)&next(e.sbit1)&!next(e.sbit2))) )  & \
			 []( ((e.sbit0 & e.sbit1 & e.sbit2) & (s.bit0 & s.bit1 & s.bit2)) -> ((next(e.sbit0)&next(e.sbit1)&next(e.sbit2))) )  & \
			 []( ((!e.sbit0 & !e.sbit1 & !e.sbit2) & (!s.bit0 & !s.bit1 & s.bit2)) -> ((!next(e.sbit0)&!next(e.sbit1)&!next(e.sbit2)) | (!next(e.sbit0)&!next(e.sbit1)&next(e.sbit2))) ) & \
			 []( ((!e.sbit0 & !e.sbit1 & !e.sbit2) & (s.bit0 & !s.bit1 & s.bit2)) -> ((!next(e.sbit0)&!next(e.sbit1)&!next(e.sbit2)) | (next(e.sbit0)&!next(e.sbit1)&next(e.sbit2))) ) & \
			 []( ((!e.sbit0 & !e.sbit1 & !e.sbit2) & (s.bit0 & s.bit1 & !s.bit2)) -> ((!next(e.sbit0)&!next(e.sbit1)&!next(e.sbit2)) | (next(e.sbit0)&next(e.sbit1)&!next(e.sbit2))) ) & \
			 []( ((!e.sbit0 & !e.sbit1 & !e.sbit2) & (s.bit0 & s.bit1 & s.bit2)) -> ((!next(e.sbit0)&!next(e.sbit1)&!next(e.sbit2)) | (next(e.sbit0)&next(e.sbit1)&next(e.sbit2))) ) & \
			 []( ((!e.sbit0 & !e.sbit1 & e.sbit2) & (!s.bit0 & !s.bit1 & !s.bit2)) -> ((!next(e.sbit0)&!next(e.sbit1)&next(e.sbit2)) | (!next(e.sbit0)&!next(e.sbit1)&!next(e.sbit2))) ) & \
			 []( ((!e.sbit0 & !e.sbit1 & e.sbit2) & (!s.bit0 & s.bit1 & s.bit2)) -> ((!next(e.sbit0)&!next(e.sbit1)&next(e.sbit2)) | (!next(e.sbit0)&next(e.sbit1)&next(e.sbit2))) ) & \
			 []( ((!e.sbit0 & e.sbit1 & !e.sbit2) & (!s.bit0 & s.bit1 & s.bit2)) -> ((!next(e.sbit0)&next(e.sbit1)&!next(e.sbit2)) | (!next(e.sbit0)&next(e.sbit1)&next(e.sbit2))) ) & \
			 []( ((!e.sbit0 & e.sbit1 & e.sbit2) & (!s.bit0 & !s.bit1 & s.bit2)) -> ((!next(e.sbit0)&next(e.sbit1)&next(e.sbit2)) | (!next(e.sbit0)&!next(e.sbit1)&next(e.sbit2))) ) & \
			 []( ((!e.sbit0 & e.sbit1 & e.sbit2) & (!s.bit0 & s.bit1 & !s.bit2)) -> ((!next(e.sbit0)&next(e.sbit1)&next(e.sbit2)) | (!next(e.sbit0)&next(e.sbit1)&!next(e.sbit2))) ) & \
			 []( ((!e.sbit0 & e.sbit1 & e.sbit2) & (s.bit0 & !s.bit1 & !s.bit2)) -> ((!next(e.sbit0)&next(e.sbit1)&next(e.sbit2)) | (next(e.sbit0)&!next(e.sbit1)&!next(e.sbit2))) ) & \
			 []( ((!e.sbit0 & e.sbit1 & e.sbit2) & (s.bit0 & s.bit1 & s.bit2)) -> ((!next(e.sbit0)&next(e.sbit1)&next(e.sbit2)) | (next(e.sbit0)&next(e.sbit1)&next(e.sbit2))) ) & \
			 []( ((e.sbit0 & !e.sbit1 & !e.sbit2) & (!s.bit0 & s.bit1 & s.bit2)) -> ((next(e.sbit0)&!next(e.sbit1)&!next(e.sbit2)) | (!next(e.sbit0)&next(e.sbit1)&next(e.sbit2))) ) & \
			 []( ((e.sbit0 & !e.sbit1 & e.sbit2) & (!s.bit0 & !s.bit1 & !s.bit2)) -> ((next(e.sbit0)&!next(e.sbit1)&next(e.sbit2)) | (!next(e.sbit0)&!next(e.sbit1)&!next(e.sbit2))) ) & \
			 []( ((e.sbit0 & e.sbit1 & !e.sbit2) & (!s.bit0 & !s.bit1 & !s.bit2)) -> ((next(e.sbit0)&next(e.sbit1)&!next(e.sbit2)) | (!next(e.sbit0)&!next(e.sbit1)&!next(e.sbit2))) ) & \
			 []( ((e.sbit0 & e.sbit1 & e.sbit2) & (!s.bit0 & !s.bit1 & !s.bit2)) -> ((next(e.sbit0)&next(e.sbit1)&next(e.sbit2)) | (!next(e.sbit0)&!next(e.sbit1)&!next(e.sbit2))) ) & \
			 []( ((e.sbit0 & e.sbit1 & e.sbit2) & (!s.bit0 & s.bit1 & s.bit2)) -> ((next(e.sbit0)&next(e.sbit1)&next(e.sbit2)) | (!next(e.sbit0)&next(e.sbit1)&next(e.sbit2))) ) & \
			 []( ((!next(e.sbit0)&!next(e.sbit1)&!next(e.sbit2))) <-> ! ( (!next(e.sbit0)&!next(e.sbit1)&next(e.sbit2)) | (!next(e.sbit0)&next(e.sbit1)&!next(e.sbit2)) | (!next(e.sbit0)&next(e.sbit1)&next(e.sbit2)) | (next(e.sbit0)&!next(e.sbit1)&!next(e.sbit2)) | (next(e.sbit0)&!next(e.sbit1)&next(e.sbit2)) | (next(e.sbit0)&next(e.sbit1)&!next(e.sbit2)) | (next(e.sbit0)&next(e.sbit1)&next(e.sbit2))) )  & \
			 []( ((!next(e.sbit0)&!next(e.sbit1)&next(e.sbit2))) <-> ! ( (!next(e.sbit0)&!next(e.sbit1)&!next(e.sbit2)) | (!next(e.sbit0)&next(e.sbit1)&!next(e.sbit2)) | (!next(e.sbit0)&next(e.sbit1)&next(e.sbit2)) | (next(e.sbit0)&!next(e.sbit1)&!next(e.sbit2)) | (next(e.sbit0)&!next(e.sbit1)&next(e.sbit2)) | (next(e.sbit0)&next(e.sbit1)&!next(e.sbit2)) | (next(e.sbit0)&next(e.sbit1)&next(e.sbit2))) )  & \
			 []( ((!next(e.sbit0)&next(e.sbit1)&!next(e.sbit2))) <-> ! ( (!next(e.sbit0)&!next(e.sbit1)&!next(e.sbit2)) | (!next(e.sbit0)&!next(e.sbit1)&next(e.sbit2)) | (!next(e.sbit0)&next(e.sbit1)&next(e.sbit2)) | (next(e.sbit0)&!next(e.sbit1)&!next(e.sbit2)) | (next(e.sbit0)&!next(e.sbit1)&next(e.sbit2)) | (next(e.sbit0)&next(e.sbit1)&!next(e.sbit2)) | (next(e.sbit0)&next(e.sbit1)&next(e.sbit2))) )  & \
			 []( ((!next(e.sbit0)&next(e.sbit1)&next(e.sbit2))) <-> ! ( (!next(e.sbit0)&!next(e.sbit1)&!next(e.sbit2)) | (!next(e.sbit0)&!next(e.sbit1)&next(e.sbit2)) | (!next(e.sbit0)&next(e.sbit1)&!next(e.sbit2)) | (next(e.sbit0)&!next(e.sbit1)&!next(e.sbit2)) | (next(e.sbit0)&!next(e.sbit1)&next(e.sbit2)) | (next(e.sbit0)&next(e.sbit1)&!next(e.sbit2)) | (next(e.sbit0)&next(e.sbit1)&next(e.sbit2))) )  & \
			 []( ((next(e.sbit0)&!next(e.sbit1)&!next(e.sbit2))) <-> ! ( (!next(e.sbit0)&!next(e.sbit1)&!next(e.sbit2)) | (!next(e.sbit0)&!next(e.sbit1)&next(e.sbit2)) | (!next(e.sbit0)&next(e.sbit1)&!next(e.sbit2)) | (!next(e.sbit0)&next(e.sbit1)&next(e.sbit2)) | (next(e.sbit0)&!next(e.sbit1)&next(e.sbit2)) | (next(e.sbit0)&next(e.sbit1)&!next(e.sbit2)) | (next(e.sbit0)&next(e.sbit1)&next(e.sbit2))) )  & \
			 []( ((next(e.sbit0)&!next(e.sbit1)&next(e.sbit2))) <-> ! ( (!next(e.sbit0)&!next(e.sbit1)&!next(e.sbit2)) | (!next(e.sbit0)&!next(e.sbit1)&next(e.sbit2)) | (!next(e.sbit0)&next(e.sbit1)&!next(e.sbit2)) | (!next(e.sbit0)&next(e.sbit1)&next(e.sbit2)) | (next(e.sbit0)&!next(e.sbit1)&!next(e.sbit2)) | (next(e.sbit0)&next(e.sbit1)&!next(e.sbit2)) | (next(e.sbit0)&next(e.sbit1)&next(e.sbit2))) )  & \
			 []( ((next(e.sbit0)&next(e.sbit1)&!next(e.sbit2))) <-> ! ( (!next(e.sbit0)&!next(e.sbit1)&!next(e.sbit2)) | (!next(e.sbit0)&!next(e.sbit1)&next(e.sbit2)) | (!next(e.sbit0)&next(e.sbit1)&!next(e.sbit2)) | (!next(e.sbit0)&next(e.sbit1)&next(e.sbit2)) | (next(e.sbit0)&!next(e.sbit1)&!next(e.sbit2)) | (next(e.sbit0)&!next(e.sbit1)&next(e.sbit2)) | (next(e.sbit0)&next(e.sbit1)&next(e.sbit2))) )  & \
			 []( ((next(e.sbit0)&next(e.sbit1)&next(e.sbit2))) <-> ! ( (!next(e.sbit0)&!next(e.sbit1)&!next(e.sbit2)) | (!next(e.sbit0)&!next(e.sbit1)&next(e.sbit2)) | (!next(e.sbit0)&next(e.sbit1)&!next(e.sbit2)) | (!next(e.sbit0)&next(e.sbit1)&next(e.sbit2)) | (next(e.sbit0)&!next(e.sbit1)&!next(e.sbit2)) | (next(e.sbit0)&!next(e.sbit1)&next(e.sbit2)) | (next(e.sbit0)&next(e.sbit1)&!next(e.sbit2))) )  & \
			 []( (!e.sbit0 & !e.sbit1 & !e.sbit2) \
				 | (!e.sbit0 & !e.sbit1 & e.sbit2)\
				 | (!e.sbit0 & e.sbit1 & !e.sbit2)\
				 | (!e.sbit0 & e.sbit1 & e.sbit2)\
				 | (e.sbit0 & !e.sbit1 & !e.sbit2)\
				 | (e.sbit0 & !e.sbit1 & e.sbit2)\
				 | (e.sbit0 & e.sbit1 & !e.sbit2)\
				 | (e.sbit0 & e.sbit1 & e.sbit2)\
			) &'

checkSpecWithInputsOutputs(spec, '../examples/FStwo_robot_negotiation/city/wander/bob/bob_wander.ltl',input_propositions, output_propositions, sensor_dict, sensor_dict)



