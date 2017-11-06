# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
informUser, 1
checkObjectWithinReach, 1
tryLocateObject, 1
planPath, 1
executePath, 1
pickupDone, 1

CompileOptions:
neighbour_robot: False
convexify: True
parser: structured
symbolic: False
use_region_bit_encoding: True
multi_robot_mode: negotiation
cooperative_gr1: True
fastslow: True
only_realizability: False
recovery: False
include_heading: False
winning_livenesses: False
synthesizer: slugs
decompose: True
interactive: True

CurrentConfigName:
Untitled configuration

Customs: # List of custom propositions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
locatedObject, 1
objectWithinReach, 1
objectInHand, 1
pathFound, 1


======== SPECIFICATION ========

GlobalSensors: # Sensors accessible by all robots

OtherRobot: # The other robot in the same workspace

Spec: # Specification in structured English
####### GOAL ######
# try to grab an object #
#################
#always r2

#### prop definition #####
# locatedObject: locate object and in memory publish objectLocation
# objectWithinReach: preplan path
# pickup: only take preplan path then execute
# objectInHand: check if object is in your hand
# replan path: prepare for pickup again



#CASE 0:  first locate object
do tryLocateObject

#FAILURE0: cannot locate object
if you have finished tryLocateObject and you are not sensing locatedObject then do informUser

#CASE 1:  check if object is within reach (checkObjectWithinReach affects objectWithinReach )
if you have finished tryLocateObject and you are sensing  locatedObject then do checkObjectWithinReach

# related actions
if you are activating tryLocateObject then visit  (locatedObject or not locatedObject)
if you are activating checkObjectWithinReach then visit (objectWithinReach or not objectWithinReach)

#FAILURE1: object is out of reach
if you are sensing locatedObject and finished checkObjectWithinReach and not objectWithinReach then do informUser

#CASE 2: object is within reach
if you are sensing locatedObject and objectWithinReach then do planPath

#FAILURE2: cannot find a path
if you have finished planPath and you are not sensing pathFound then do informUser

#CASE 3: find path, execute it
if you are sensing pathFound then do executePath

#FAILURE2: did not grasp the object
if you have finished executePath and you are not sensing objectInHand then do informUser

#CASE 4: pickup completed
if you have finished executePath and you are sensing objectInHand then do pickupDone

