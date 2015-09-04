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
../map_new.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
alice_gov, 1
alice_roadBR, 1
alice_roadTR, 1
alice_roadBL, 1
alice_roadTL, 1
alice_office2, 1
alice_office1, 1


======== SPECIFICATION ========

OtherRobot: # The other robot in the same workspace
alice

RegionMapping: # Mapping between region names and their decomposed counterparts
roadTL = p2
roadBL = p4
roadBR = p3
office2 = p6
office1 = p7
others = 
roadTR = p1
gov = p8

Spec: # Specification in structured English
Robot starts in roadTR
Environment starts with alice_office1

#############
## Patrol ###
#############
visit roadTR
visit roadBR

#####################################
# ASSUMPTIONS ABOUT THE OTHER ROBOT #
#####################################
if you have finished office1 then do not (alice_office1 or alice_office2)
if you have finished office2 then do not (alice_office2 or alice_office1 or alice_roadTL or alice_roadBL)
if you have finished roadTL then do not (alice_roadTL or alice_roadTR or alice_office2)
if you have finished roadBL then do not (alice_roadBL or alice_roadBR or alice_office2)
if you have finished roadTR then do not (alice_roadTR or alice_roadTL or alice_gov)
if you have finished roadBR then do not (alice_roadBR or alice_roadBL or alice_gov)
if you have finished gov then do not (alice_gov or alice_roadBR or alice_roadTR)

