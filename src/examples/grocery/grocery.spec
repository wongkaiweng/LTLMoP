# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
callManager, 1
gripperClosed, 1
alarm, 1

CompileOptions:
convexify: True
parser: structured
symbolic: False
use_region_bit_encoding: True
synthesizer: jtlv
fastslow: False
decompose: True

CurrentConfigName:
basicSim

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
grocery.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
blockage, 1
missingItem, 1
reporting, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
r1 = p8
r2 = p7
r3 = p6
office = p9
others = p1, p2, p3, p4

Spec: # Specification in structured English
group Corners is r1, r2, r3, office
robot starts in any Corner with false

# shout out for the notice
do alarm if and only if you are sensing blockage
if you are sensing blockage then stay there

# Call the manager when you find a missing item
callManager is set on missingItem and reset on reporting

# Pick up the missingitem and send it to the office
if you are activating callManager then do gripperClosed
if you are activating start of callManager then stay there
if you are activating callManager then visit office

# drop item to manger
if you are activating end of callManager then do not gripperClosed

# stay in place if asked
if you are sensing reporting then stay there
infinitely often not (reporting or blockage)

# Patrol the store
if you are not activating callManager then visit all Corners

