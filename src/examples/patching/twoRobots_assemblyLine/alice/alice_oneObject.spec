# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
paint_block, 1

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
interactive: False

CurrentConfigName:
basicSim

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
../twoRegions.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
bob_tookBlock, 1
blockPresent, 1
bob_r2, 1
bob_r1, 1


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

# stay in place and do one task at a time
always finished bob_r2
if you had finished r1 then do r1
#always ((paint_block and not paint_disk and not paint_triangle) or (not paint_block and paint_disk and not paint_triangle) or (not paint_block and not paint_disk and paint_triangle))
#always ((paint_block and not paint_disk) or (not paint_block and paint_disk))
#always ((bob_tookBlock and not bob_tookDisk and not bob_tookTriangle) or (not bob_tookBlock and bob_tookDisk and not bob_tookTriangle) or (not bob_tookBlock and not bob_tookDisk and bob_tookTriangle))


if you are sensing blockPresent then do not bob_tookBlock

#infinitely often finished paint_block and paint_block
# paint objects
if you are sensing blockPresent then visit paint_block

# don't do the same task
if you are sensing bob_tookBlock then do not paint_block

