# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
pickup, 1
deliver, 1

CompileOptions:
neighbour_robot: False
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
orderPickedUp

RegionFile: # Relative path of region description file
../map_new2.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
order, 1


======== SPECIFICATION ========

OtherRobot: # The other robot in the same workspace
bob

RegionMapping: # Mapping between region names and their decomposed counterparts
roadTL = p2
roadBL = p4
roadBR = p3
office1 = p6
others = 
roadTR = p1
gov = p7

Spec: # Specification in structured English
Robot starts in office1

############
## ORDER  ##
############
# track if an order is recevied
orderPickedUp is set on finished pickup and finished office1 and reset on finished deliver and finished gov

###########
## pickup #
###########
if you are activating orderPickedUp then go to gov
if you are sensing order and you have finished office1 then do pickup

# stay in place to pickup
#if you are activating pickup and you have not finished pickup and you have finished office1 then do office1

###########
# deliver #
###########
# go to gov and deliver order
#if you are activating foodCooked then go to commonArea
do deliver if and only if you are activating orderPickedUp and you have finished gov
# make sure the robot don't randomly deliver
if you are not activating orderPickedUp and you have not finished gov then do not deliver

###########
## Goals ##
###########
if you are not activating orderPickedUp then visit office1

#####################################
# ASSUMPTIONS ABOUT THE OTHER ROBOT #
#####################################
#if you have finished office1 then do not (bob_office1 or bob_roadTL or bob_roadBL)
#if you have finished roadTL then do not (bob_roadTL or bob_roadTR or bob_office1)
#if you have finished roadBL then do not (bob_roadBL or bob_roadBR or bob_office1)
#if you have finished roadTR then do not (bob_roadTR or bob_roadTL or bob_gov)
#if you have finished roadBR then do not (bob_roadBR or bob_roadBL or bob_gov)
#if you have finished gov then do not (bob_gov or bob_roadBR or bob_roadTR)

