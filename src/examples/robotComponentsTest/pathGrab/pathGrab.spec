# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
leftForward, 1
rightForward, 1
indicateHaveObject, 1
indicateComplete, 1
secureObject, 1
releaseObject, 1

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
followPath is set on (not atObject and (not secureObject or releaseObject)) or (secureObject and not atGoal) and reset on (atObject and releaseObject) or (atGoal and releaseObject)

# Follow the path
do leftForward if and only if (not onPath) and followPath
do rightForward if and only if onPath and followPath

# Grasp object when reached and release it when at goal
do secureObject if and only if (atObject or secureObject) and not atGoal
do releaseObject if and only if atGoal

# Indicate when object is grasped
do indicateHaveObject if and only if secureObject

# Indicate when task is complete
do indicateComplete if and only if (atGoal and releaseObject)

if you are activating secureObject then do not (releaseObject)
if you are activating releaseObject then do not (secureObject)

