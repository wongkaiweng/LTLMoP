# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)

CompileOptions:
neighbour_robot: True
convexify: True
parser: structured
symbolic: False
use_region_bit_encoding: True
multi_robot_mode: patching
fastslow: True
recovery: False
include_heading: False
synthesizer: slugs
decompose: True

CurrentConfigName:
basicSim

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
../map.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
bob_gov, 1
bob_threeWay, 1
bob_roadBR, 1
bob_roadTR, 1
bob_roadBL, 1
bob_roadTL, 1
bob_office2, 1
bob_office1, 1


======== SPECIFICATION ========

OtherRobot: # The other robot in the same workspace
bob

RegionMapping: # Mapping between region names and their decomposed counterparts
roadTL = p3
roadBL = p5
roadBR = p4
threeWay = p1
office1 = p8
others = 
roadTR = p2
gov = p9
office2 = p7

Spec: # Specification in structured English
Robot starts in office1
Environment starts with bob_gov
############
## ORDER  ##
############
# track if the waiter informs us that we should cook food
#if you are not activating orderReceived then go to kitchen
#orderReceived is set on waiter_order and reset on finished deliver and finished commonArea
#orderReceived is set on waiterOrder and reset on finished deliver and finished commonArea

###########
## pickup #
###########
# cook food in kitchen
#if you are activating orderReceived and you are not activating foodCooked then go to kitchen
#if you are activating orderReceived and you have finished kitchen then do cook
#foodCooked is set on finished cook and finished kitchen and reset on finished deliver and finished commonArea

###########
# deliver #
###########
# go back to commonArea and hang over food
#if you are activating foodCooked then go to commonArea
#do deliver if and only if you are activating foodCooked and you have finished commonArea
# make sure the robot don't randomly deliver
#if you are not activating foodCooked and you have not finished commonArea then do not deliver

# assume we never get out of the commonArea
#always not finished order and not finished hallway and not finished counter and not finished customer

###########
## Goals ##
###########
visit gov

#####################################
# ASSUMPTIONS ABOUT THE OTHER ROBOT #
#####################################
if you have finished office1 then do not (bob_office1 or bob_office2)
if you have finished office2 then do not (bob_office2 or bob_office1 or bob_roadTL or bob_roadBL)
if you have finished roadTL then do not (bob_roadTL or bob_roadTR or bob_office2)
if you have finished roadBL then do not (bob_roadBL or bob_roadBR or bob_office2)
if you have finished roadTR then do not (bob_roadTR or bob_roadTL or bob_threeWay)
if you have finished roadBR then do not (bob_roadBR or bob_roadBL or bob_threeWay)
if you have finished threeWay then do not (bob_threeWay or bob_roadBR or bob_roadTR or bob_gov)
if you have finished gov then do not (bob_gov or bob_threeWay)

