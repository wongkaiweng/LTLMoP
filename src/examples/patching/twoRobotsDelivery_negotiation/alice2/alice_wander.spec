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
office = p7
roadC = p3
roadB = p4
roadA = p5
storage = p1
roadD = p2
others = 

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
if you have finished office then do not (bob_office or bob_roadA or bob_roadC)
if you have finished roadA then do not (bob_roadA or bob_roadB or bob_office)
if you have finished roadC then do not (bob_roadC or bob_roadD or bob_office)
if you have finished roadB then do not (bob_roadB or bob_roadA or bob_storage)
if you have finished roadD then do not (bob_roadD or bob_roadC or bob_storage)
if you have finished storage then do not (bob_storage or bob_roadB or bob_roadC)

### extra sysTrans ####
if you are sensing bob_office then do not office
if you are sensing bob_roadA then do not roadA
if you are sensing bob_roadB then do not roadB
if you are sensing bob_roadC then do not roadC
if you are sensing bob_roadD then do not roadD
if you are sensing bob_storage then do not storage

