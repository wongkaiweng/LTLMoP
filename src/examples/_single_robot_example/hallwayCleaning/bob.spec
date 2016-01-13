# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
sweeping, 1

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
onlyBob

Customs: # List of custom propositions
sweptTop
sweptBottom
sweptCentral

RegionFile: # Relative path of region description file
../../../../../../../../../home/catherine/LTLMoP/src/examples/FStwo_robot_negotiation/delivery/threeCorridorsShort.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
obstacle, 1
cleanArea, 1


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
Robot starts in office

infinitely often not cleanArea and not obstacle
if you are sensing obstacle then stay there

if you are not activating (sweptTop or sweptBottom or sweptCentral) or you are activating (sweptTop and sweptBottom and sweptCentral) then visit office
if you are not activating sweptTop and you are sensing cleanArea then visit hallwayTop
if you are not activating sweptBottom and you are sensing cleanArea then visit hallwayBottom
if you are not activating sweptCentral and you are sensing cleanArea then visit hallwayCentral

do sweeping if and only if you are sensing cleanArea and  (( not sweptTop and finished hallwayTop )  or ( not sweptBottom and finished hallwayBottom) or ( not sweptCentral and finished hallwayCentral ))
sweptTop is set on (finished sweeping and finished hallwayTop) and reset on not cleanArea
sweptBottom is set on finished sweeping and finished hallwayBottom and reset on not cleanArea
sweptCentral is set on finished sweeping and finished hallwayCentral and reset on not cleanArea

