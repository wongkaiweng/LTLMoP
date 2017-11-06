# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
moveToBox, 1

CompileOptions:
neighbour_robot: False
convexify: False
parser: structured
symbolic: False
use_region_bit_encoding: False
multi_robot_mode: negotiation
cooperative_gr1: True
fastslow: True
only_realizability: True
recovery: False
include_heading: False
winning_livenesses: False
synthesizer: slugs
decompose: False
interactive: True

CurrentConfigName:
Untitled configuration

Customs: # List of custom propositions
moveLeft
moveRight
moveUp
moveDown

RegionFile: # Relative path of region description file
../../../../../../LTLMoP/src/examples/actuations/spheros/spheros.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
packageTask, 1
moveBoxLeft, 1
moveBoxRight, 1
moveBoxUp, 1
moveBoxDown, 1
ready, 1
noOtherHeadsToR1, 1
noOtherHeadsToR2, 1
noOtherHeadsToR3, 1
noOtherHeadsToR4, 1


======== SPECIFICATION ========

GlobalSensors: # Sensors accessible by all robots

OtherRobot: # The other robot in the same workspace

RegionMapping: # Mapping between region names and their decomposed counterparts
r4 = p1
r1 = p4
r2 = p3
r3 = p2
atirum = p5
others = 

Spec: # Specification in structured English
# move to box if you get a task
If you are sensing packageTask and not ready then do moveToBox

# move box once you get the package
If you are sensing packageTask and moveBoxLeft and ready then do moveLeft
If you are sensing packageTask and moveBoxRight and ready then do moveRight
If you are sensing packageTask and moveBoxUp and ready then do moveUp
If you are sensing packageTask and moveBoxDown and ready then do moveDown


# patrol when there's no package
if you are sensing noOtherHeadsToR1 and not packageTask   then visit r1
if you are sensing noOtherHeadsToR2 and not packageTask  then visit r2
if you are sensing noOtherHeadsToR3 and not packageTask   then visit r3
if you are sensing noOtherHeadsToR4 and not packageTask   then visit r4

