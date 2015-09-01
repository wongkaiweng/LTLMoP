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
../restaurant_new.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
waiter_order, 1
waiter_customer, 1
waiter_hallway, 1
waiter_counter, 1
waiter_commonArea, 1
waiter_kitchen, 1
waiter_entrance, 1
waiter_waitingArea, 1


======== SPECIFICATION ========

OtherRobot: # The other robot in the same workspace
waiter

RegionMapping: # Mapping between region names and their decomposed counterparts
customer = p7
hallway = p4
entrance = p6
commonArea = p9
counter = p8
order = p2
others = 
waitingArea = p1
kitchen = p3

Spec: # Specification in structured English
Robot starts in kitchen

######################
# Goals of the robot #
######################
visit customer

#####################################
# ASSUMPTIONS ABOUT THE OTHER ROBOT #
#####################################
if you have finished customer then do not (waiter_customer or waiter_hallway)
if you have finished hallway then do not (waiter_hallway or waiter_order or waiter_entrance)
if you have finished entrance then do not (waiter_entrance or waiter_waitingArea)
if you have finished waitingArea then do not (waiter_waitingArea or  waiter_counter)
if you have finished counter then do not (waiter_counter or waiter_waitingArea or waiter_commonArea)
if you have finished order then do not (waiter_order or waiter_commonArea)
if you have finished commonArea then do not (waiter_commonArea or waiter_kitchen)
if you have finished kitchen then do not (waiter_kitchen)

#if you have finished customer then do not (waiter_customer or waiter_hallway)
#if you have finished hallway then do not (waiter_hallway or waiter_customer or waiter_order or waiter_entrance)
#if you have finished entrance then do not (waiter_entrance or waiter_hallway or waiter_waitingArea)
#if you have finished waitingArea then do not (waiter_waitingArea or waiter_entrance or waiter_counter)
#if you have finished counter then do not (waiter_counter or waiter_waitingArea or waiter_commonArea)
#if you have finished order then do not (waiter_order or waiter_hallway or waiter_commonArea)
#if you have finished commonArea then do not (waiter_commonArea or waiter_counter or waiter_order or waiter_kitchen)
#if you have finished kitchen then do not (waiter_kitchen or waiter_commonArea)

