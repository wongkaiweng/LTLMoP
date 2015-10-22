# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)

CompileOptions:
neighbour_robot: True
convexify: False
parser: structured
symbolic: False
use_region_bit_encoding: True
multi_robot_mode: patching
fastslow: True
recovery: False
include_heading: False
synthesizer: slugs
decompose: True

CurrentConfigName:
basicSim

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
../patching_new.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
alice_hallway2, 1
charlie_hallway2, 1
alice_hallway1, 1
charlie_hallway1, 1
alice_exit, 1
charlie_exit, 1
alice_commonAreaR, 1
charlie_commonAreaR, 1
alice_commonAreaL, 1
charlie_commonAreaL, 1
alice_entrance, 1
charlie_entrance, 1
alice_storageR, 1
charlie_storageR, 1
alice_roomR, 1
charlie_roomR, 1
alice_office, 1
charlie_office, 1
alice_storageL, 1
charlie_storageL, 1
alice_library, 1
charlie_library, 1
alice_roomL, 1
charlie_roomL, 1
alice_station, 1
charlie_station, 1
alice_hallway3, 1
charlie_hallway3, 1
alice_hallway4, 1
charlie_hallway4, 1


======== SPECIFICATION ========

OtherRobot: # The other robot in the same workspace
alice
charlie

RegionMapping: # Mapping between region names and their decomposed counterparts
entrance = p13
storageL = p2
office = p6
commonAreaL = p15
hallway2 = p10
hallway3 = p9
roomL = p5
hallway1 = p11
hallway4 = p8
station = p3
roomR = p4
library = p7
commonAreaR = p14
exit = p12
storageR = p1
others = 

Spec: # Specification in structured English
# env init
Robot starts in library
Environment starts with alice_roomL and charlie_office

# environment assumptions
if you have finished library then do not (alice_library or charlie_library or alice_entrance or charlie_entrance)
if you have finished entrance then do not (alice_entrance or charlie_entrance or alice_hallway1 or charlie_hallway1 or alice_roomL or charlie_roomL)
if you have finished roomL then do not (alice_roomL or charlie_roomL or alice_entrance or charlie_entrance or alice_hallway1 or charlie_hallway1)
if you have finished hallway1 then do not (alice_hallway1 or charlie_hallway1 or alice_commonAreaL or charlie_commonAreaL or alice_roomL or charlie_roomL)
if you have finished hallway3 then do not (alice_hallway3 or charlie_hallway3 or alice_storageL or charlie_storageL or alice_hallway1 or charlie_hallway1)
if you have finished storageL then do not (alice_storageL or charlie_storageL or alice_hallway3 or charlie_hallway3)

######### system guarantees ##########
# not allowing both robots in the same region
#if you are sensing alice_r1 then do not r1
#if you are sensing alice_r2 then do not r2
#if you are sensing alice_r3 then do not r3
#if you are sensing alice_r4 then do not r4
#if you are sensing alice_r5 then do not r5

# system goals #
visit library
visit storageL

