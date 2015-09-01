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
chef_commonArea, 1
chef_kitchen, 1
chef_order, 1
chef_counter, 1
chef_hallway, 1
chef_customer, 1
chef_entrance, 1
chef_waitingArea, 1


======== SPECIFICATION ========

OtherRobot: # The other robot in the same workspace
chef

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
Robot starts in customer

############
## goals  ##
############
visit kitchen

#####################################
# ASSUMPTIONS ABOUT THE OTHER ROBOT #
#####################################
if you have finished customer then do not (chef_customer or chef_hallway)
if you have finished hallway then do not (chef_hallway or chef_order or chef_entrance)
if you have finished entrance then do not (chef_entrance or chef_waitingArea)
if you have finished waitingArea then do not (chef_waitingArea or chef_counter)
if you have finished counter then do not (chef_counter or chef_commonArea)
if you have finished order then do not (chef_order or  chef_commonArea)
if you have finished commonArea then do not (chef_commonArea or chef_kitchen)
if you have finished kitchen then do not (chef_kitchen)

#if you have finished customer then do not (chef_customer or chef_hallway)
#if you have finished hallway then do not (chef_hallway or chef_customer or chef_order or chef_entrance)
#if you have finished entrance then do not (chef_entrance or chef_hallway or chef_waitingArea)
#if you have finished waitingArea then do not (chef_waitingArea or chef_entrance or chef_counter)
#if you have finished counter then do not (chef_counter or chef_waitingArea or chef_commonArea)
#if you have finished order then do not (chef_order or chef_hallway or  chef_commonArea)
#if you have finished commonArea then do not (chef_commonArea or chef_counter or chef_order or chef_kitchen)
#if you have finished kitchen then do not (chef_kitchen or chef_commonArea)

