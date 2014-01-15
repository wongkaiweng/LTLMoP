# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
pickup, 1
deliver, 1

CompileOptions:
convexify: True
parser: structured
fastslow: False
decompose: True
use_region_bit_encoding: True

CurrentConfigName:
Nao_maecy

Customs: # List of custom propositions
obtainedPackage
betweenClasses

RegionFile: # Relative path of region description file
packageDelivery.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
packageReady, 1
doorClosed, 1
cooking, 1
classStarts, 1
classIsOver, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
classroom = p20, p21
hallway = p6
door = p18, p19
office = p12, p13
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

# system safety guarantees
If you were sensing doorClosed then do not door
If you were sensing cooking then do not kitchen
If you are activating betweenClasses then do not atrium
betweenClasses is set on classStarts and reset on classIsOver
if you are sensing classStarts then stay there
if you are sensing classIsOver then stay there

# pick up package in the postOffice
If you are sensing packageReady then visit mailroom
do pickup if and only if you are in mailroom and you are sensing packageReady and not obtainedPackage
if you were activating obtainedPackage then do not mailroom
obtainedPackage is set on (mailroom and packageReady and pickup) and reset on (office and deliver)

# deliver package to the workplace
If you are sensing obtainedPackage then go to office
do deliver if and only If you are in office and you are sensing obtainedPackage

