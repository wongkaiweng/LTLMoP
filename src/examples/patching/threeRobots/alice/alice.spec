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
../letterHwithExtraMiddle.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
bob_M1, 1
charlie_M1, 1
bob_R3, 1
charlie_R3, 1
bob_R2, 1
charlie_R2, 1
bob_R4, 1
charlie_R4, 1
bob_R5, 1
charlie_R5, 1
bob_R1, 1
charlie_R1, 1
bob_ML, 1
charlie_ML, 1
bob_MR, 1
charlie_MR, 1
bob_L4, 1
charlie_L4, 1
bob_L2, 1
charlie_L2, 1
bob_L5, 1
charlie_L5, 1
bob_L3, 1
charlie_L3, 1
bob_L1, 1
charlie_L1, 1


======== SPECIFICATION ========

OtherRobot: # The other robot in the same workspace
bob
charlie

RegionMapping: # Mapping between region names and their decomposed counterparts
R4 = p2
R5 = p1
R1 = p5
R2 = p4
R3 = p3
ML = p7
L1 = p13
L4 = p10
L5 = p9
M1 = p8
L3 = p11
MR = p6
L2 = p12
others = 

Spec: # Specification in structured English
Robot starts in L3
Environment starts with bob_R3 and charlie_M1
# and not waveHand_ac

######### env goals #############

####### env assumptions  #########
always not (bob_L1 or bob_L2 or bob_L3 or bob_L4 or bob_L5)
always not (charlie_L1 or charlie_L2 or charlie_L3 or charlie_L4 or charlie_L5)
#if you have finished r1 then do alice_r4
#if you have finished r2 then do alice_r5
#if you have finished r3 then do alice_r6
#if you have finished r4 then do alice_r1
#if you have finished r5 then do alice_r2

######## system guarantees #######
#if you are sensing bob_R1 then do not (R1 or R2) and (R3 or ML or M1 or MR)
#if you are sensing alice_r4 then do not finish r5

######### system goals ##########
visit L5
visit L1

