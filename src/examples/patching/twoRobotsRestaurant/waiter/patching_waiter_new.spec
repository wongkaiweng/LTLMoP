# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
pickup, 1
deliver, 1

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
foodInHand
foodNeeded

RegionFile: # Relative path of region description file
../restaurant_new.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
foodOrder, 1
foodReady, 1
chef_commonArea, 1
chef_prepArea, 1
chef_kitchen, 1
chef_order, 1
chef_counter, 1
chef_hallway, 1
chef_customer, 1
chef_entrance, 1


======== SPECIFICATION ========

OtherRobot: # The other robot in the same workspace
chef

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
Robot starts in customer

############
## ORDER  ##
############
# wait for order in the customer region
if you are not activating foodNeeded then go to customer
# track if customer has ordered and go to order to inform CHEF
foodNeeded is set on foodOrder and finished customer and reset on finished deliver and finished customer
if you are activating foodNeeded and you are not activating foodInHand and you are not sensing foodReady then go to order

###########
## pickup #
###########
# go to commonArea to pick up food
if you are sensing foodReady then go to commonArea
if you are sensing foodReady and you have finished commonArea then do pickup
foodInHand is set on foodReady and finished pickup and finished commonArea and reset on finished deliver and finished customer

###########
# deliver #
###########
# go back to customer and deliver food
if you are activating foodInHand then go to customer
do deliver if and only if you are activating foodInHand and you have finished customer
# make sure the robot don't randomly deliver
if you are not activating foodInHand and you have not finished customer then do not deliver
# don't go back to the order region when you have food.
if you are activating foodInHand then do not finish order

always not finished kitchen
#####################################
# ASSUMPTIONS ABOUT THE OTHER ROBOT #
#####################################
if you have finished customer then do not (chef_customer or chef_hallway)
if you have finished hallway then do not (chef_hallway or chef_customer or chef_order or chef_entrance)
if you have finished entrance then do not (chef_entrance or chef_hallway or chef_counter)
if you have finished counter then do not (chef_counter or chef_hallway or chef_commonArea)
if you have finished order then do not (chef_order or chef_hallway or  chef_commonArea)
if you have finished commonArea then do not (chef_commonArea or chef_counter or chef_order or chef_kitchen)
#if you have finished kitchen then do not (chef_kitchen or chef_commonArea or chef_prepArea)
#if you have finished prepArea then do not (chef_prepArea or chef_kitchen)

#always do not finish customer
#infinitely often not foodInHand

