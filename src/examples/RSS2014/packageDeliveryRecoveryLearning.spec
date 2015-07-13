# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
pickup, 1
deliver, 1

CompileOptions:
synthesizer: jtlv
neighbour_robot: False
fastslow: False
include_heading: False
convexify: False
recovery: False
parser: structured
symbolic: False
decompose: True
use_region_bit_encoding: True

CurrentConfigName:
BasicSim

Customs: # List of custom propositions
obtainedPackage

RegionFile: # Relative path of region description file
packageDelivery.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
packageReady, 1
doorClosed, 1
cooking, 1
betweenClasses, 1


======== SPECIFICATION ========

OtherRobot: # The other robot in the same workspace

RegionMapping: # Mapping between region names and their decomposed counterparts
classroom = p10
hallway = p6
door = p8
office = p3
mailroom = p4
corridor = p9
others = 
atrium = p11
kitchen = p5

Spec: # Specification in structured English
# inital conditions of the system and the environment
Env starts with false
Robot starts with false
Robot starts in hallway

always not doorClosed
if you were sensing cooking then do not cooking
if you were sensing betweenClasses then do not betweenClasses
# system safety guarantees
If you are sensing doorClosed then do not door
If you are sensing cooking then do not kitchen
If you are activating betweenClasses then do not atrium

# pick up package in the postOffice
If you are sensing packageReady and you are not activating obtainedPackage then visit mailroom
do pickup if and only if you are in mailroom and you are sensing packageReady and you are not activating obtainedPackage

#if you are sensing packageReady then do not mailroom
if you were activating obtainedPackage then do not mailroom
obtainedPackage is set on pickup and reset on deliver

# deliver package to the workplace
if you are activating obtainedPackage then go to office
do deliver if and only If you are in office and you are activating obtainedPackage

