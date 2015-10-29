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
multi_robot_mode: d-patching
cooperative_gr1: True
fastslow: True
only_realizability: False
recovery: False
include_heading: False
winning_livenesses: True
synthesizer: slugs
decompose: True
interactive: False

CurrentConfigName:
basicSim

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
../map_new2.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
alice_storage, 1
alice_roadD, 1
alice_roadB, 1
alice_roadC, 1
alice_roadA, 1
alice_office, 1


======== SPECIFICATION ========

OtherRobot: # The other robot in the same workspace
alice

RegionMapping: # Mapping between region names and their decomposed counterparts
office = p7
roadC = p3
roadB = p4
roadA = p5
storage = p1
roadD = p2
others = 

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
if you have finished office then do not (alice_office or alice_roadA or alice_roadC)
if you have finished roadA then do not (alice_roadA or alice_roadB or alice_office)
if you have finished roadC then do not (alice_roadC or alice_roadD or alice_office)
if you have finished roadB then do not (alice_roadB or alice_roadA or alice_storage)
if you have finished roadD then do not (alice_roadD or alice_roadC or alice_storage)
if you have finished storage then do not (alice_storage or alice_roadD or alice_roadB)

### extra sysTrans ####
if you are sensing alice_office then do not office
if you are sensing alice_roadA then do not roadA
if you are sensing alice_roadB then do not roadB
if you are sensing alice_roadC then do not roadC
if you are sensing alice_roadD then do not roadD
if you are sensing alice_storage then do not storage

