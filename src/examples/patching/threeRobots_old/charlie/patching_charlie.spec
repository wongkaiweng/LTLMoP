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
bob_hallway2, 1
alice_hallway2, 1
bob_hallway1, 1
alice_hallway1, 1
bob_exit, 1
alice_exit, 1
bob_commonAreaR, 1
alice_commonAreaR, 1
bob_commonAreaL, 1
alice_commonAreaL, 1
bob_entrance, 1
alice_entrance, 1
bob_storageR, 1
alice_storageR, 1
bob_roomR, 1
alice_roomR, 1
bob_office, 1
alice_office, 1
bob_storageL, 1
alice_storageL, 1
bob_library, 1
alice_library, 1
bob_roomL, 1
alice_roomL, 1
bob_station, 1
alice_station, 1
bob_hallway4, 1
alice_hallway4, 1
bob_hallway3, 1
alice_hallway3, 1


======== SPECIFICATION ========

OtherRobot: # The other robot in the same workspace
bob
alice

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
Robot starts in office
Environment starts with bob_library and alice_roomL

# environment assumptions
if you have finished office then do not (bob_office or alice_office or bob_exit or alice_exit)
if you have finished exit then do not (bob_exit or alice_exit or bob_office or alice_office or bob_roomR or alice_roomR)
if you have finished roomR then do not (bob_roomR or alice_roomR or bob_exit or alice_exit or bob_hallway2 or alice_hallway2)
if you have finished hallway2 then do not (bob_hallway2 or alice_hallway2 or bob_hallway4 or alice_hallway4 or bob_roomR or alice_roomR)
if you have finished hallway4 then do not (bob_hallway4 or alice_hallway4 or bob_hallway2 or alice_hallway2 or bob_storageR or alice_storageR)
if you have finished storageR then do not (bob_storageR or alice_storageR or bob_hallway4 or alice_hallway4)


######### system guarantees ##########
# not allowing both robots in the same region
#if you are sensing bob_r1 then do not r1
#if you are sensing bob_r2 then do not r2
#if you are sensing bob_r3 then do not r3
#if you are sensing bob_r4 then do not r4
#if you are sensing bob_r5 then do not r5

# system goals #
visit storageR
visit office

