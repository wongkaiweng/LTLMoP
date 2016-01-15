# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
placeBlock, 1

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
interactive: False

CurrentConfigName:
basicSim

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
../blockCutting.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
alice_r2, 1
alice_r1, 1
blockPresent, 1
alice_holdingBlock, 1


======== SPECIFICATION ========

GlobalSensors: # Sensors accessible by all robots

OtherRobot: # The other robot in the same workspace
alice

RegionMapping: # Mapping between region names and their decomposed counterparts
r1 = p2
r2 = p1
others = 

Spec: # Specification in structured English
Robot starts in r2
Env starts with alice_r1

always alice_r1

# this should come from alice
#if you are not sensing blockPresent then do placeBlock


#do placeBlock if and only if you are sensing not blockPresent

#if you have finished placeBlock then do blockPresent

if you are sensing blockPresent and not alice_holdingBlock then do not placeBlock
if you are sensing alice_holdingBlock then do placeBlock

