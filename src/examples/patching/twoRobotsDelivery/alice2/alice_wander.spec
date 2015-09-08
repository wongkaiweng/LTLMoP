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
../map_new2.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
bob_storage, 1
bob_roadSouth, 1
bob_roadEast, 1
bob_roadWest, 1
bob_roadNorth, 1
bob_office, 1


======== SPECIFICATION ========

OtherRobot: # The other robot in the same workspace
bob

RegionMapping: # Mapping between region names and their decomposed counterparts
roadEast = p5
office = p7
roadSouth = p3
storage = p1
others = 
roadWest = p2
roadNorth = p4

Spec: # Specification in structured English
Robot starts in office
Environment starts with bob_storage

###########
## Goals ##
###########
visit storage

#####################################
# ASSUMPTIONS ABOUT THE OTHER ROBOT #
#####################################
if you have finished office then do not (bob_office or bob_roadNorth or bob_roadWest)
if you have finished roadNorth then do not (bob_roadNorth or bob_roadEast or bob_office)
if you have finished roadWest then do not (bob_roadWest or bob_roadSouth or bob_office)
if you have finished roadEast then do not (bob_roadEast or bob_roadNorth or bob_storage)
if you have finished roadSouth then do not (bob_roadSouth or bob_roadWest or bob_storage)
if you have finished storage then do not (bob_storage or bob_roadEast or bob_roadWest)

