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
alice_M1, 1
charlie_M1, 1
alice_R2, 1
charlie_R2, 1
alice_R3, 1
charlie_R3, 1
alice_R1, 1
charlie_R1, 1
alice_ML, 1
charlie_ML, 1
alice_MR, 1
charlie_MR, 1
alice_L3, 1
charlie_L3, 1
alice_L1, 1
charlie_L1, 1
alice_L2, 1
charlie_L2, 1


======== SPECIFICATION ========

OtherRobot: # The other robot in the same workspace
alice
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
Robot starts in R2
Environment starts with alice_L2 and charlie_M1

always not (alice_R1 or alice_R2 or alice_R3)
always not (charlie_R1 or charlie_R2 or charlie_R3)

visit R1
visit R3

