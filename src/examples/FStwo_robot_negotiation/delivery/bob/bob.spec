# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
sweeping, 1

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
cleanArea, 0
alice_hallwayTop, 1
alice_office, 1
alice_reception, 1
alice_cafe, 1
alice_emergencyExit, 1
alice_hallwayBottom, 1
alice_hallwayCentral, 1
alice_library, 1
alice_atrium, 1
alice_storageBottom, 1
alice_storageTop, 1
trashInHallwayTop, 1
trashInHallwayCentral, 1
trashInHallwayBottom, 1
itemReceived, 1


======== SPECIFICATION ========

GlobalSensors: # Sensors accessible by all robots

OtherRobot: # The other robot in the same workspace
alice

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
Robot starts in hallwayCentral
Environment starts with alice_library

always (not trashInHallwayTop and not trashInHallwayCentral and not trashInHallwayBottom) or (trashInHallwayTop and not trashInHallwayCentral and not trashInHallwayBottom) or (not trashInHallwayTop and trashInHallwayCentral and not trashInHallwayBottom) or (not trashInHallwayTop and not trashInHallwayCentral and trashInHallwayBottom)

if you are sensing trashInHallwayTop then visit hallwayTop
#if you are sensing trashInHallwayBottom then visit hallwayBottom
if you are sensing trashInHallwayCentral  or  not (trashInHallwayTop or trashInHallwayBottom or trashInHallwayCentral)  then visit hallwayCentral

do sweeping if and only if you are sensing (( trashInHallwayTop and finished hallwayTop )  or ( trashInHallwayBottom and finished hallwayBottom) or (trashInHallwayCentral and finished hallwayCentral ))

# other robot's assumptions
#if you are sensing cleanArea and (finished emergencyExit or finished storageTop) then do not alice_hallwayTop

#always alice_library

#if you have finished library then do not (alice_library or alice_atrium)
if you have finished atrium then do not (alice_atrium or alice_storageTop or alice_storageBottom or alice_hallwayCentral)
if you have finished storageTop then do not (alice_atrium or alice_storageTop or alice_hallwayTop)
if you have finished storageBottom then do not (alice_atrium or alice_storageBottom or alice_hallwayBottom)
if you have finished hallwayCentral then do not (alice_atrium or alice_hallwayCentral or alice_reception)
if you have finished hallwayTop then do not (alice_storageTop or alice_hallwayTop or alice_emergencyExit)
if you have finished hallwayBottom then do not (alice_storageBottom or alice_hallwayBottom or alice_cafe)
if you have finished emergencyExit then do not (alice_hallwayTop or alice_emergencyExit or alice_reception)
if you have finished cafe then do not (alice_hallwayBottom or alice_cafe or alice_reception)
if you have finished reception then do not (alice_hallwayCentral or alice_reception or alice_emergencyExit or alice_cafe)
#if you have finished office then do not (alice_office or alice_reception)

