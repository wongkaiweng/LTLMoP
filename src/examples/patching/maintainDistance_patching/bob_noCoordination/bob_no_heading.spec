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
multi_robot_mode: d-patching
cooperative_gr1: True
fastslow: True
only_realizability: False
recovery: False
include_heading: False
winning_livenesses: False
synthesizer: slugs
decompose: True
interactive: True

CurrentConfigName:
naoInLab

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
../loop_six_regions.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
alice_r1, 1
alice_r2, 1
alice_r3, 1
alice_r4, 1
alice_r5, 1
alice_r6, 1


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
Robot starts in r4
Environment starts with alice_r1
# and not waveHand_ac

######### env goals #############
#infinitely often not  camera
#infinitely often not alice_r2 and finished r2
#infinitely often not alice_r3 and finished r3
#infinitely often not alice_r4 and finished r4
#infinitely often not alice_r5 and finished r5

####### env assumptions  #########
#if you were sensing waveHand_ac and you were sensing alice_waveHand then do not camera
if you have finished r1 then do alice_r4
if you have finished r2 then do alice_r5
if you have finished r3 then do alice_r6
if you have finished r4 then do alice_r1
if you have finished r5 then do alice_r2

######## system guarantees #######
if you are sensing alice_r1 then do not (r6 or r1 or r2) and (r3 or r4 or r5)
if you are sensing alice_r2 then do not (r1 or r2 or r3) and (r4 or r5 or r6)
if you are sensing alice_r3 then do not (r2 or r3 or r4) and (r5 or r6 or r1)
if you are sensing alice_r4 then do not (r3 or r4 or r5) and (r6 or r1 or r2)
if you are sensing alice_r5 then do not (r4 or r5 or r6) and (r1 or r2 or r3)
if you are sensing alice_r6 then do not (r5 or r6 or r1) and (r2 or r3 or r4)

#if you are sensing alice_r4 then do not finish r5
#if you are sensing alice_r3 then do not finish r4
#if you are sensing alice_r2 then do not finish r3

######### system goals ##########
visit r3
#visit r4
#visit r2

