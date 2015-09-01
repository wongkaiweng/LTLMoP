# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
deliver, 1
cook, 1

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
orderReceived
foodCooked

RegionFile: # Relative path of region description file
../restaurant_new.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
waiterOrder, 0
waiter_order, 1
waiter_customer, 1
waiter_hallway, 1
waiter_counter, 1
waiter_commonArea, 1
waiter_kitchen, 1
waiter_prepArea, 1
waiter_entrance, 1


======== SPECIFICATION ========

OtherRobot: # The other robot in the same workspace
waiter

RegionMapping: # Mapping between region names and their decomposed counterparts
customer = p6
hallway = p3
entrance = p5
commonArea = p8
counter = p7
others = 
order = p1
kitchen = p2

Spec: # Specification in structured English
Robot starts in kitchen

############
## ORDER  ##
############
# track if the waiter informs us that we should cook food
if you are not activating orderReceived then go to kitchen
orderReceived is set on waiter_order and reset on finished deliver and finished commonArea
#orderReceived is set on waiterOrder and reset on finished deliver and finished commonArea

###########
## pickup #
###########
# cook food in kitchen
if you are activating orderReceived and you are not activating foodCooked then go to kitchen
if you are activating orderReceived and you have finished kitchen then do cook
foodCooked is set on finished cook and finished kitchen and reset on finished deliver and finished commonArea

###########
# deliver #
###########
# go back to commonArea and hang over food
if you are activating foodCooked then go to commonArea
do deliver if and only if you are activating foodCooked and you have finished commonArea
# make sure the robot don't randomly deliver
if you are not activating foodCooked and you have not finished commonArea then do not deliver

# assume we never get out of the commonArea
always not finished order and not finished hallway and not finished counter and not finished customer

#####################################
# ASSUMPTIONS ABOUT THE OTHER ROBOT #
#####################################
#if you have finished customer then do not (waiter_customer or waiter_hallway)
#if you have finished hallway then do not (waiter_hallway or waiter_customer or waiter_order or waiter_counter)
#if you have finished counter then do not (waiter_counter or waiter_hallway or waiter_order or waiter_commonArea)
#if you have finished order then do not (waiter_order or waiter_hallway or waiter_counter or waiter_commonArea)
if you have finished commonArea then do not (waiter_commonArea or waiter_counter or waiter_order or waiter_kitchen)
if you have finished kitchen then do not (waiter_kitchen or waiter_commonArea or waiter_prepArea)
#if you have finished prepArea then do not (waiter_prepArea or waiter_kitchen)

#always do not finish customer
#infinitely often not foodInHand

