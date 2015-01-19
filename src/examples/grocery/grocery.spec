# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
lookLeftright, 1
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
pickupItem
dropItem
noticeOut

RegionFile: # Relative path of region description file
grocery.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
seeMissingitem, 1
headTapped, 1
seeNotice, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
r1 = p6
between$r1$and$r2$ = p9
r3 = p4
office = p7
between$r3$and$office$ = p8
r2 = p5
others = p1, p2

Spec: # Specification in structured English
group Corners is r1, r2, r3, office
robot starts in any Corner with false

# Remember notices you've detected
noticeOut is set on (r1 or r2 or r3 or office) and seeNotice and reset on ((between r1 and r2) or (between r3 and office)) and not seeNotice
if start of noticeOut then stay there

# shout out for the notice
do alarm if and only if start of noticeOut

# Call the manager when you find a missing item
callManager is set on seeMissingitem and reset on headTapped
# Pick up the missingitem and send it to the office
if you are activating start of callManager then stay there
if you are activating start of callManager then do pickupItem
if you are activating callManager then visit office

# drop item to manger
if you are activating end of callManager then do dropItem
gripperClosed is set on pickupItem and reset on dropItem

# stay in place if asked
if you are sensing headTapped then stay there
infinitely often not headTapped

# Patrol the store
if you are not activating callManager then visit all Corners

# look around next to the shelves
do lookLeftright if and only if you are not activating callManager and you were in (between r1 and r2 or between r3 and office)

