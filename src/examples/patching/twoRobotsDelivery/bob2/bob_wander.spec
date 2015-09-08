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
alice_storage, 1
alice_roadSouth, 1
alice_roadEast, 1
alice_roadWest, 1
alice_roadNorth, 1
alice_office, 1


======== SPECIFICATION ========

OtherRobot: # The other robot in the same workspace
alice

RegionMapping: # Mapping between region names and their decomposed counterparts
roadEast = p5
office = p7
roadSouth = p3
storage = p1
others = 
roadWest = p2
roadNorth = p4

Spec: # Specification in structured English
Robot starts in storage
Environment starts with alice_office

###########
## Goals ##
###########
#visit office2
visit office

#####################################
# ASSUMPTIONS ABOUT THE OTHER ROBOT #
#####################################
if you have finished office then do not (alice_office or alice_office or alice_roadNorth or alice_roadWest)
if you have finished roadNorth then do not (alice_roadNorth or alice_roadEast or alice_office)
if you have finished roadWest then do not (alice_roadWest or alice_roadSouth or alice_office)
if you have finished roadEast then do not (alice_roadEast or alice_roadNorth or alice_storage)
if you have finished roadSouth then do not (alice_roadSouth or alice_roadWest or alice_storage)
if you have finished storage then do not (alice_storage or alice_roadSouth or alice_roadEast)

