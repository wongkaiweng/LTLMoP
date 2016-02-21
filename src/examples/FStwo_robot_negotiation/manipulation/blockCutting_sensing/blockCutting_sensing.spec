# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
pickup, 1
cutBlock, 1

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
pickupTask, 1
blockPresent, 1
holdingBlock, 1
bob_r2, 1
bob_r1, 1
bob_ActplaceBlock, 1


======== SPECIFICATION ========

GlobalSensors: # Sensors accessible by all robots

OtherRobot: # The other robot in the same workspace
bob

RegionMapping: # Mapping between region names and their decomposed counterparts
r1 = p2
r2 = p1
others = 

Spec: # Specification in structured English
Robot starts in r1
Env starts with bob_r2

always bob_r2

# --#infinitely often (pickupTask and finished pickup)
# --infinitely often (holdingBlock and finished cutBlock)

#always (holdingBlock and finished cutBlock) or (pickupTask and finished pickup) or not pickupTask or (pickupTask and blockPresent)

#always not bob_placeBlock and blockPresent
always bob_ActplaceBlock or blockPresent

#and blockPresent

# assume block is always present
#if you are sensing pickupTask and not holdingBlock then do blockPresent

# --if you are sensing bob_placeBlock then do blockPresent
# --if you have finished pickup then do holdingBlock

do pickup if and only if you are sensing pickupTask and blockPresent
do cutBlock if and only if you are sensing holdingBlock


# --infinitely often cutBlock
#or not pickupTask

