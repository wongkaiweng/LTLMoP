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
symbolic: True
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
orderPickedUp

RegionFile: # Relative path of region description file
../map_new2.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
order, 1
bob_storage, 1
bob_roadD, 1
bob_roadB, 1
bob_roadC, 1
bob_roadA, 1
bob_office, 1


======== SPECIFICATION ========

OtherRobot: # The other robot in the same workspace
bob

RegionMapping: # Mapping between region names and their decomposed counterparts
roadB = p5
office = p7
roadD = p3
storage = p1
others = 
roadC = p2
roadA = p4

Spec: # Specification in structured English
Robot starts in office
Environment starts with bob_roadB

############
## ORDER  ##
############
# track if an order is recevied
orderPickedUp is set on finished pickup and finished office and reset on finished deliver and finished storage

###########
## pickup #
###########
if you are activating orderPickedUp then go to storage
do pickup if and only if you are sensing order and you have finished office and you are not activating orderPickedUp

# stay in place to pickup
if you are activating pickup and you have not finished pickup and you have finished office then finished office

###########
# deliver #
###########
# go to storage and deliver order
do deliver if and only if you are activating orderPickedUp and you have finished storage
# stay in place to pickup
if you are activating deliver and you have not finished deliver and you have finished storage then finished storage

###########
## Goals ##
###########
if you are not activating orderPickedUp then visit office

#####################################
# ASSUMPTIONS ABOUT THE OTHER ROBOT #
#####################################
if you have finished office then do not (bob_office or bob_roadA or bob_roadC)
if you have finished roadA then do not (bob_roadA or bob_roadB or bob_office)
if you have finished roadC then do not (bob_roadC or bob_roadD or bob_office)
if you have finished roadB then do not (bob_roadB or bob_roadA or bob_storage)
if you have finished roadD then do not (bob_roadD or bob_roadC or bob_storage)
if you have finished storage then do not (bob_storage or bob_roadD or bob_roadB)

