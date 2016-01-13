# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)

CompileOptions:
neighbour_robot: True
convexify: True
parser: structured
symbolic: False
use_region_bit_encoding: True
multi_robot_mode: negotiation
cooperative_gr1: True
fastslow: True
only_realizability: False
recovery: False
include_heading: True
winning_livenesses: False
synthesizer: slugs
decompose: True
interactive: False

CurrentConfigName:
bobWithAliceSensorR1

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
../two_robot_negotiation.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
alice_r1, 1
alice_r2, 1
alice_r3, 1
alice_r4, 1
alice_r5, 1
alice_r3_rc, 1
alice_r5_rc, 1
alice_r2_rc, 1
alice_r4_rc, 1
alice_r1_rc, 1


======== SPECIFICATION ========

GlobalSensors: # Sensors accessible by all robots

OtherRobot: # The other robot in the same workspace
alice

RegionMapping: # Mapping between region names and their decomposed counterparts
r4 = p3
r5 = p2
r6 = p1
r1 = p6
r2 = p5
r3 = p4
others = 

Spec: # Specification in structured English
####### initial conditions ##########
Robot starts in r1
Environment starts with alice_r3 and alice_r3_rc

###### environment assumptions ######
if you were activating r2 then do not (alice_r4_rc)
if you were activating r4 then do not (alice_r4_rc or alice_r5_rc)

#if you had finished r2 then do not (alice_r2_rc or alice_r4_rc)
#if you had finished r4 then do not (alice_r4_rc or alice_r5_rc)

if you had finished r1 then do not (alice_r1 or alice_r2)
if you had finished r2 then do not (alice_r2 or alice_r4)
if you had finished r4 then do not (alice_r4 or alice_r5)
if you had finished r5 then do not alice_r5

######### system guarantees ##########
# not allowing both robots to be at the same place
if you are sensing alice_r1_rc then do not r1
if you are sensing alice_r2_rc then do not r2
if you are sensing alice_r3_rc then do not r3
if you are sensing alice_r4_rc then do not r4
if you are sensing alice_r5_rc then do not r5

# not allowing both robots to head to the same place
if you are sensing alice_r1 then do not r1
if you are sensing alice_r2 then do not r2
if you are sensing alice_r3 then do not r3
if you are sensing alice_r4 then do not r4
if you are sensing alice_r5 then do not r5

######## system goals ###########
visit r5

