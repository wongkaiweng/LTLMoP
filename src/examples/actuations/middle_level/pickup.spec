# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
pickup, 1
informUser, 1
replanPath, 1
checkObjectWithinReach, 1

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
startPickupAction
pickupFailure

RegionFile: # Relative path of region description file
../../_RegionFiles/one_line_two_regions.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
orderRequest, 1
locatedObject, 1
objectWithinReach, 1
objectInHand, 1


======== SPECIFICATION ========

GlobalSensors: # Sensors accessible by all robots

OtherRobot: # The other robot in the same workspace

RegionMapping: # Mapping between region names and their decomposed counterparts
r1 = p2
r2 = p1
others = 

Spec: # Specification in structured English
####### GOAL ######
# try to grab an object #
#################
always r2

#### prop definition #####
# locatedObject: locate object and in memory publish objectLocation
# objectWithinReach: preplan path
# pickup: only take preplan path then execute
# objectInHand: check if object is in your hand
# replan path: prepare for pickup again

# sense an orderRequest then initiate pickup
startPickupAction is set on orderRequest and reset on objectInHand and finished pickup

# track if pickup is successful
pickupFailure is set on not objectInHand and finished pickup and reset on objectInHand and finished pickup

#CASE 1: object is within reach
if you are sensing startPickupAction and locatedObject and checkObjectWithinReach and objectWithinReach and not pickupFailure then do pickup

# check if object is within reach (checkObjectWithinReach affects objectWithinReach )
if you are sensing startPickupAction and locatedObject then do checkObjectWithinReach
if you are activating checkObjectWithinReach then visit (objectWithinReach or not objectWithinReach)

#CASE 2: object is out of reach
if you are sensing startPickupAction and locatedObject and finished checkObjectWithinReach and not objectWithinReach then do informUser

#CASE 3: initiated pickup but pickup failed
if you are sensing startPickupAction and pickupFailure then do replanPath
if you are sensing startPickupAction and pickupFailure and replanPath and you have not finished replanPath  then do not pickup
if you are sensing startPickupAction and pickupFailure and replanPath and you have finished replanPath then do pickup

