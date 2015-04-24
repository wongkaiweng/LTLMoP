# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
moveToSource, 1
indicateComplete, 1
closeGripper, 1

CompileOptions:
convexify: True
parser: structured
symbolic: False
use_region_bit_encoding: True
synthesizer: jtlv
fastslow: False
decompose: True

CurrentConfigName:
Untitled configuration

Customs: # List of custom propositions
waitingForObject

RegionFile: # Relative path of region description file
grasper_simulate.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
seeObject, 1
masterCalling, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
r1 = p2
others = p3, p4, p5, p6

Spec: # Specification in structured English

robot starts with not waitingForObject

# Wait for an object after user summons
waitingForObject is set on masterCalling and reset on closeGripper

# Wait at the source for the object
do moveToSource if and only if waitingForObject
# Close gripper when object is detected at source
do closeGripper if and only if moveToSource and seeObject
# Indicate completion
do indicateComplete if and only if not closeGripper and not moveToSource

