# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
pickup, 0
deliver, 0

CompileOptions:
convexify: False
parser: structured
symbolic: False
use_region_bit_encoding: True
synthesizer: False
fastslow: False
decompose: True

CurrentConfigName:
BasicSim

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
packageDelivery.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
packageReady, 0
doorClosed, 1
cooking, 0
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
robot starts in mailroom
visit corridor
if you are sensing doorClosed then do not door
if you are in hallway then do not doorClosed

