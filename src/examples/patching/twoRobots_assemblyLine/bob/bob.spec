# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
takeBlock, 1
stamp, 1

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
blockStamped

RegionFile: # Relative path of region description file
../twoRegions.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
alice_r2, 1
alice_r1, 1
alice_takeBlock, 1
startStamping, 1


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

# stay in place and do one task at a time
always not alice_r2
if you had finished r2 then do r2
#always ((paint_block and not paint_disk and not paint_triangle) or (not paint_block and paint_disk and not paint_triangle) or (not paint_block and not paint_disk and paint_triangle))
#always ((paint_block and not paint_disk) or (not paint_block and paint_disk))
#always ((alice_tookBlock and not alice_tookDisk and not alice_tookTriangle) or (not alice_tookBlock and alice_tookDisk and not alice_tookTriangle) or (not alice_tookBlock and not alice_tookDisk and alice_tookTriangle))

infinitely often startStamping and blockStamped
#  the painting job
do takeBlock if and only if you are not sensing alice_takeBlock and you are not sensing blockStamped and you are sensing startStamping
do stamp if and only if you have finished takeBlock
if you are sensing startStamping then visit blockStamped
blockStamped is set on finished stamp and finished takeBlock and reset on not startStamping

# don't take the same block
#if you are sensing alice_takeBlock then do not takeBlock
if you are activating takeBlock then do not alice_takeBlock

