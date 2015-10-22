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
cooperative_gr1: True
fastslow: True
only_realizability: False
recovery: False
include_heading: False
winning_livenesses: False
synthesizer: slugs
decompose: True
interactive: False

CurrentConfigName:
basicSim

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
../patching_new.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
bob_hallway2, 1
charlie_hallway2, 1
bob_hallway1, 1
charlie_hallway1, 1
bob_exit, 1
charlie_exit, 1
bob_commonAreaR, 1
charlie_commonAreaR, 1
bob_commonAreaL, 1
charlie_commonAreaL, 1
bob_entrance, 1
charlie_entrance, 1
bob_storageR, 1
charlie_storageR, 1
bob_roomR, 1
charlie_roomR, 1
bob_office, 1
charlie_office, 1
bob_storageL, 1
charlie_storageL, 1
bob_library, 1
charlie_library, 1
bob_roomL, 1
charlie_roomL, 1
bob_station, 1
charlie_station, 1
bob_hallway4, 1
charlie_hallway4, 1
bob_hallway3, 1
charlie_hallway3, 1


======== SPECIFICATION ========

OtherRobot: # The other robot in the same workspace
bob
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
Robot starts in roomL
Environment starts with bob_library and charlie_office

# environment assumptions
if you have finished roomL then do not (bob_roomL or charlie_roomL or bob_hallway1 or charlie_hallway1)
if you have finished hallway1 then do not (bob_hallway1 or charlie_hallway1 or bob_commonAreaL or charlie_commonAreaL or bob_roomL or charlie_roomL)
if you have finished commonAreaL then do not (bob_commonAreaL or charlie_commonAreaL or bob_station or charlie_station or bob_hallway1 or charlie_hallway1)
if you have finished station then do not (bob_station or charlie_station or bob_commonAreaL or charlie_commonAreaL or bob_commonAreaR or charlie_commonAreaR)
if you have finished commonAreaR then do not (bob_commonAreaR or charlie_commonAreaR or bob_hallway2 or charlie_hallway2 or bob_station or charlie_station)
if you have finished hallway2 then do not (bob_hallway2 or charlie_hallway2 or bob_commonAreaR or charlie_commonAreaR or bob_roomR or charlie_roomR)
if you have finished roomR then do not (bob_roomR or charlie_roomR or bob_hallway2 or charlie_hallway2)

######### system guarantees ##########
# not allowing both robots in the same region
#if you are sensing bob_r1 then do not r1
#if you are sensing bob_r2 then do not r2
#if you are sensing bob_r3 then do not r3
#if you are sensing bob_r4 then do not r4
#if you are sensing bob_r5 then do not r5

# system goals #
visit roomL
visit station
visit roomR

