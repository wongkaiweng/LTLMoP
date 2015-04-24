# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
leftForward, 1
rightForward, 1
gripperClosed, 1
indicateHaveObject, 1
indicateComplete, 1

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
followPath

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
onPath, 1
atObject, 1
atGoal, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
r1 = p2
others = p3, p4, p5, p6

Spec: # Specification in structured English
robot starts with false

# Follow the path to get to the object and to move with the object
followPath is set on (not atObject and not gripperClosed) or (gripperClosed and not atGoal) and reset on (atObject and not gripperClosed) or (atGoal and not gripperClosed)

# Follow the path
do leftForward if and only if (not onPath) and followPath
do rightForward if and only if onPath and followPath

# Grasp object when reached and release it when at goal
do gripperClosed if and only if (atObject or gripperClosed) and not atGoal

# Indicate when object is grasped
do indicateHaveObject if and only if gripperClosed

# Indicate when task is complete
do indicateComplete if and only if (atGoal and not gripperClosed)

