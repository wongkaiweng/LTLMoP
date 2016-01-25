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
alice_M1, 1
bob_M1, 1
alice_R2, 1
bob_R2, 1
alice_R3, 1
bob_R3, 1
alice_R1, 1
bob_R1, 1
alice_ML, 1
bob_ML, 1
alice_MR, 1
bob_MR, 1
alice_L3, 1
bob_L3, 1
alice_L2, 1
bob_L2, 1
alice_L1, 1
bob_L1, 1
requestL3, 1
requestR1, 1


======== SPECIFICATION ========

GlobalSensors: # Sensors accessible by all robots

OtherRobot: # The other robot in the same workspace
alice
bob

RegionMapping: # Mapping between region names and their decomposed counterparts
R1 = p3
R2 = p2
R3 = p1
ML = p5
L1 = p9
M1 = p6
L3 = p7
MR = p4
L2 = p8
others = 

Spec: # Specification in structured English
Robot starts in M1
Environment starts with alice_L2 and bob_R2

# some assumptions to make it realizable
# only mantain distance when needed?

#-- restrict regions alice and bob can be in)
#always not (alice_M1 or alice_ML or alice_MR or alice_R1 or alice_R1 or alice_R2 or alice_R3 or alice_R5)
#always not (bob_M1 or bob_ML or bob_MR or bob_L1 or bob_L1 or bob_L2 or bob_L3 or bob_L5)

#always  (alice_M1 or alice_ML or alice_MR or alice_L1 or alice_L1 or alice_L2 or alice_L3 or alice_L5)
#always  (bob_M1 or bob_ML or bob_MR or bob_R1 or bob_R1 or bob_R2 or bob_R3 or bob_R5)
#if you are not sensing requestL3 then do not (alice_M1 or alice_ML or alice_MR)
#if you are not sensing requestR1 then do not (bob_M1 or bob_ML or bob_MR)

always  (alice_L1 or alice_L2 or alice_L3)
always  (bob_R1 or bob_R2 or bob_R3)

always not (requestL3 and requestR1)
# --- maintain distance when visiting L5 --- #
#if you are activating requestL3 and you have finished M1 then do not (alice_ML or alice_M1 or alice_MR or alice_L2)
#if you are activating requestL3 and you have finished ML then do not (alice_L2 or alice_ML or alice_MR or alice_M1 or alice_L1 or alice_L3)
#if you are activating requestL3 and you have finished L2 then do not (alice_L2 or alice_L1 or alice_L3 or alice_L1 or alice_L5 or alice_MR or alice_M1 or alice_ML)
#if you are activating requestL3 and you have finished L3 then do not (alice_L2 or alice_L1 or alice_L3 or alice_L5 or alice_ML)
#if you are activating requestL3 and you have finished L2 then do not (alice_L2 or alice_L1 or alice_L3 or alice_L1 or alice_L5 or alice_MR or alice_M1 or alice_ML)

#if you have finished M1 then do not (alice_ML or alice_M1 or alice_MR)
#if you have finished ML then do not (alice_ML or alice_MR or alice_M1)
if you are activating requestL3 and you have finished ML then do not  alice_L2
if you have finished L2 then do not (alice_L2 or alice_L3)
if you have finished L3 then do not (alice_L2 or alice_L3)
#if you have finished L1 then do not (alice_L2 or alice_L1)

#if you have finished M1 then do not (bob_ML or bob_M1 or bob_MR)
#if you have finished MR then do not (bob_ML or bob_MR or bob_M1)
if you are activating requestR1 and you have finished MR then do not  bob_R2
if you have finished R2 then do not (bob_R2 or bob_R1)
#if you have finished R3 then do not (bob_R2 or bob_R3)
if you have finished R1 then do not (bob_R2 or bob_R1)

####### system goals ########
if you are sensing requestL3 then visit L3
if you are sensing requestR1 then visit R1
if you are not sensing (requestL3 or requestR1) then visit M1

