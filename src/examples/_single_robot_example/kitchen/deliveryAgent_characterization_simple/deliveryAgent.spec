# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)

CompileOptions:
neighbour_robot: False
convexify: True
parser: structured
symbolic: False
use_region_bit_encoding: True
multi_robot_mode: negotiation
cooperative_gr1: True
fastslow: True
only_realizability: False
recovery: True
include_heading: False
winning_livenesses: False
synthesizer: slugs
decompose: True
interactive: False

CurrentConfigName:
Untitled configuration

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
../../../../../../../../../home/catherine/LTLMoP/src/examples/_single_robot_example/kitchen/workspace_new_simple.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
order, 1
doorOpen, 1
doorBroken, 1


======== SPECIFICATION ========

GlobalSensors: # Sensors accessible by all robots

OtherRobot: # The other robot in the same workspace

RegionMapping: # Mapping between region names and their decomposed counterparts
company = p3
storage = p1
road = p2
others = 

Spec: # Specification in structured English
Robot starts in company


# deliver item to prepArea
if you are sensing order then visit storage and stay there

# the assistant needs to open the door
if you have finished road or storage then do doorOpen
if you are not sensing doorOpen or doorBroken then do not storage

# stay/go back to company when there's no order
if you are not sensing order then visit company and stay there

