# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
paint, 1
takeBlock, 1

CompileOptions:
neighbour_robot: False
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
baxter

Customs: # List of custom propositions
blockPainted

RegionFile: # Relative path of region description file
../twoRegions.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
bob_r2, 1
bob_r1, 1
bob_takeBlock, 1
startPainting, 1


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

infinitely often startPainting and blockPainted
#  the painting job
do takeBlock if and only if you are not sensing bob_takeBlock and you are not sensing blockPainted and you are sensing startPainting
do paint if and only if you have finished takeBlock
if you are sensing startPainting then visit blockPainted
blockPainted is set on finished paint and finished takeBlock and reset on not startPainting

# don't take the same block
#if you are sensing bob_takeBlock then do not takeBlock
if you are activating takeBlock then do not bob_takeBlock

