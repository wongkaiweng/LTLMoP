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
alice_M1, 1
charlie_M1, 1
alice_R3, 1
charlie_R3, 1
alice_R2, 1
charlie_R2, 1
alice_R4, 1
charlie_R4, 1
alice_R5, 1
charlie_R5, 1
alice_R1, 1
charlie_R1, 1
alice_ML, 1
charlie_ML, 1
alice_MR, 1
charlie_MR, 1
alice_L4, 1
charlie_L4, 1
alice_L2, 1
charlie_L2, 1
alice_L5, 1
charlie_L5, 1
alice_L3, 1
charlie_L3, 1
alice_L1, 1
charlie_L1, 1


======== SPECIFICATION ========

OtherRobot: # The other robot in the same workspace
alice
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
Robot starts in R3
Environment starts with alice_L3 and charlie_M1

always not (alice_R1 or alice_R2 or alice_R3 or alice_R4 or alice_R5)
always not (charlie_R1 or charlie_R2 or charlie_R3 or charlie_R4 or charlie_R5)

visit R1
visit R5

