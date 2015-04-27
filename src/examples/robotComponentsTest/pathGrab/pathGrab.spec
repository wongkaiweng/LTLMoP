# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
secureObject_d, 1
releaseObject_d, 1
indicateHaveObject_du, 1
indicateComplete_md, 1
leftForward_d, 1
rightForward_d, 1

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
atGoal_du, 1
atObject_d, 1
onPath_d, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
r1 = p2
others = p3, p4, p5, p6

Spec: # Specification in structured English
robot starts with false

# Follow the path to get to the object and to move with the object
followPath is set on (not atObject_d and (not secureObject_d or releaseObject_d)) or (secureObject_d and not atGoal_du) and reset on (atObject_d and releaseObject_d) or (atGoal_du and releaseObject_d)

# Follow the path
do leftForward_d if and only if (not onPath_d) and followPath
do rightForward_d if and only if onPath_d and followPath

# Grasp object when reached and release it when at goal
do secureObject_d if and only if (atObject_d or secureObject_d) and not atGoal_du
do releaseObject_d if and only if atGoal_du

# Indicate when object is grasped
do indicateHaveObject_du if and only if secureObject_d

# Indicate when task is complete
do indicateComplete_md if and only if (atGoal_du and releaseObject_d)

if you are activating secureObject_d then do not (releaseObject_d)
if you are activating releaseObject_d then do not (secureObject_d)

