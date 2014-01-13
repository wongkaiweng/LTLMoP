# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
pickup, 1
deliver, 1

CompileOptions:
convexify: False
parser: structured
fastslow: False
decompose: True
use_region_bit_encoding: True

CurrentConfigName:
BasicSim

Customs: # List of custom propositions
obtainedPackage

RegionFile: # Relative path of region description file
packageDelivery.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
ferryCrossing, 1
roadBlockage, 1
packageReady, 1
accident, 1
doorClosed, 0


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
bridge = p12
postOffice = p4
pavement = p5
exit = p10
workplace = p1
others = 
home = p8
crosswalk = p11
highway = p9

Spec: # Specification in structured English
# inital conditions of the system and the environment
Env starts with false
Robot starts with false
Robot starts in  home

# system safety guarantees
If you are sensing ferryCrossing then do not bridge
If you are sensing roadBlockage then do not exit
If you are sensing accident then do not crosswalk
#f you are sensing doorClosed then do not door

# pick up package in the postOffice
If you are sensing packageReady then visit postOffice
do pickup if and only if you are in postOffice and you are sensing packageReady and not obtainedPackage
obtainedPackage is set on (postOffice and packageReady and pickup) and reset on (workplace and deliver)


# deliver package to the workplace
If you are sensing obtainedPackage then go to workplace
do deliver if and only If you are in workplace and you are sensing obtainedPackage

