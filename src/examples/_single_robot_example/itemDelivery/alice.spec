# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
pickup, 1
deliver, 1

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
interactive: True

CurrentConfigName:
onlyAlice

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
../../../../../../../../home/catherine/LTLMoP/src/examples/FStwo_robot_negotiation/delivery/threeCorridorsShort.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
itemRequest, 1
blockTop, 1
blockMiddle, 1
blockBottom, 1
itemReceived, 1


======== SPECIFICATION ========

GlobalSensors: # Sensors accessible by all robots

OtherRobot: # The other robot in the same workspace

RegionMapping: # Mapping between region names and their decomposed counterparts
emergencyExit = p11
hallwayBottom = p8
office = p4
storageTop = p1
storageBottom = p2
hallwayTop = p6
library = p5
reception = p3
hallwayCentral = p7
atrium = p13
cafe = p12
others = 

Spec: # Specification in structured English
Robot starts in library

#always (not blockTop and not blockMiddle and blockBottom) or  (not blockTop and blockMiddle and not blockBottom) or (not blockTop and not blockMiddle and not blockBottom)
# or (blockTop and not blockMiddle and not blockBottom)

always not blockTop
# don't go to hallways when they are blocked
if you are sensing blockTop then do not hallwayTop
if you are sensing blockMiddle then do not hallwayCentral
if you are sensing blockBottom then do not hallwayBottom

# pick up and deliver things
if you are sensing itemRequest then do pickup
#itemReceived is set on finished pickup and reset on finished deliver
if you are sensing itemReceived then visit office
do deliver if and only if you are sensing itemReceived and you have finished office

# system goals
if you are not activating itemReceived then visit library

