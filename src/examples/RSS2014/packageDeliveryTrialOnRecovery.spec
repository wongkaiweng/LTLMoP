# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
pickup, 1
deliver, 0

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
packageReady, 0
doorClosed, 0
cooking, 1
betweenClasses, 0


======== SPECIFICATION ========

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
Env starts with false
robot starts in door
robot starts with false

if you were in door then do not  cooking
if you are in door then do not  pickup
if you are sensing cooking then do pickup

visit door and stay there

