# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
pickup, 1
deliver, 1

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
../threeCorridorsShort.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
itemRequest, 1
bob_hallwayTop, 1
bob_office, 1
bob_reception, 1
bob_cafe, 1
bob_emergencyExit, 1
bob_hallwayBottom, 1
bob_hallwayCentral, 1
bob_library, 1
bob_atrium, 1
bob_storageBottom, 1
bob_storageTop, 1
itemReceived, 1
trashInHallwayTop, 1
trashInHallwayBottom, 1
trashInHallwayCentral, 1


======== SPECIFICATION ========

GlobalSensors: # Sensors accessible by all robots

OtherRobot: # The other robot in the same workspace
bob

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
Environment starts with bob_hallwayCentral
#always (not blockTop and not blockMiddle and blockBottom) or  (not blockTop and blockMiddle and not blockBottom) or (not blockTop and not blockMiddle and not blockBottom)
# or (blockTop and not blockMiddle and not blockBottom)

# don't go to hallways when they are blocked
#if you are sensing blockTop then do not hallwayTop
#if you are sensing blockMiddle then do not hallwayCentral
#if you are sensing blockBottom then do not hallwayBottom

# pick up and deliver things
if you are sensing itemRequest then do pickup
#itemReceived is set on finished pickup and reset on finished deliver
if you are sensing itemReceived then visit office
do deliver if and only if you are sensing itemReceived and you have finished office

# system goals
if you are not activating itemReceived then visit library

# assumptions about the other robot
if you are activating itemReceived then do not (bob_library or bob_atrium or bob_storageTop or bob_hallwayTop or bob_emergencyExit or bob_reception or bob_office)
if you are not activating itemReceived then do not (bob_library or bob_atrium or bob_storageBottom or bob_hallwayBottom or bob_cafe or bob_reception or bob_office)
#always not (bob_library or bob_atrium or bob_storageTop or bob_hallwayTop or bob_emergencyExit or bob_reception or bob_office)
#always bob_hallwayCentral

if you have finished library then do not (bob_library or bob_atrium)
if you have finished atrium then do not (bob_library or bob_atrium or bob_storageTop or bob_storageBottom or bob_hallwayCentral)
if you have finished storageTop then do not (bob_atrium or bob_storageTop or bob_hallwayTop)
if you have finished storageBottom then do not (bob_atrium or bob_storageBottom or bob_hallwayBottom)
if you have finished hallwayCentral then do not (bob_atrium or bob_hallwayCentral or bob_reception)
if you have finished hallwayTop then do not (bob_storageTop or bob_hallwayTop or bob_emergencyExit)
if you have finished hallwayBottom then do not (bob_storageBottom or bob_hallwayBottom or bob_cafe)
if you have finished emergencyExit then do not (bob_hallwayTop or bob_emergencyExit or bob_reception)
if you have finished cafe then do not (bob_hallwayBottom or bob_cafe or bob_reception)
if you have finished reception then do not (bob_hallwayCentral or bob_reception or bob_emergencyExit or bob_cafe)
if you have finished office then do not (bob_office or bob_reception)

