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
basicSim

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
../letterHwithExtraMiddle_short.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
bob_M1, 1
charlie_M1, 1
bob_R2, 1
charlie_R2, 1
bob_R1, 1
charlie_R1, 1
bob_R3, 1
charlie_R3, 1
bob_ML, 1
charlie_ML, 1
bob_MR, 1
charlie_MR, 1
bob_L3, 1
charlie_L3, 1
bob_L1, 1
charlie_L1, 1
bob_L2, 1
charlie_L2, 1


======== SPECIFICATION ========

OtherRobot: # The other robot in the same workspace
bob
charlie

RegionMapping: # Mapping between region names and their decomposed counterparts
R1 = p3
R2 = p2
R3 = p1
ML = p5
MR = p4
M1 = p6
L3 = p7
L1 = p9
L2 = p8
others = 

Spec: # Specification in structured English
Robot starts in L2
Environment starts with bob_R2 and charlie_M1
# and not waveHand_ac

######### env goals #############

####### env assumptions  #########
always not (bob_L1 or bob_L2 or bob_L3)
always not (charlie_L1 or charlie_L2 or charlie_L3)
#if you have finished r1 then do alice_r3
#if you have finished r1 then do alice_r5
#if you have finished r2 then do alice_r6
#if you have finished r3 then do alice_r1
#if you have finished r5 then do alice_r1

######## system guarantees #######
#if you are sensing bob_R1 then do not (R1 or R1) and (R2 or ML or M1 or MR)
#if you are sensing alice_r3 then do not finish r5

######### system goals ##########
visit L3
visit L1

