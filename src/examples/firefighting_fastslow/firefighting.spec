# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
pick_up, 1
drop, 1
radio, 1
extinguish, 0
stay_in_place, 1

CompileOptions:
neighbour_robot: False
convexify: False
parser: structured
symbolic: False
use_region_bit_encoding: False
multi_robot_mode: negotiation
cooperative_gr1: True
fastslow: True
only_realizability: False
recovery: False
include_heading: False
winning_livenesses: False
synthesizer: slugs
decompose: False
interactive: True

CurrentConfigName:
Basic Simulation

Customs: # List of custom propositions
carrying_item

RegionFile: # Relative path of region description file
floorplan.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
fire, 0
person, 1
hazardous_item, 1


======== SPECIFICATION ========

GlobalSensors: # Sensors accessible by all robots

OtherRobot: # The other robot in the same workspace

RegionMapping: # Mapping between region names and their decomposed counterparts
living = p4
porch = p3
deck = p7
others = 
dining = p6
bedroom = p8
kitchen = p5

Spec: # Specification in structured English
# Initial conditions
#Env starts with false
Robot starts in porch

# Assumptions about the environment
If you have finished porch then do not hazardous_item
If you have finished porch then do not person

# Define robot safety including how to pick up
Do pick_up if and only if you are sensing hazardous_item and you are not activating carrying_item
#If you are activating pick_up then stay there
carrying_item is set on pick_up and finished pick_up and reset on drop and finished drop
Do drop if and only if you have finished porch and you are activating carrying_item

#If you did not activate carrying_item then always not porch

# Define when and how to radio
Do radio if and only if you are sensing person
If you are activating radio or you were activating radio then do  stay_in_place

# Patrol goals
#Group rooms is living, bedroom, deck, kitchen, dining
#If you are not activating carrying_item and you are not activating radio then visit all rooms

If you are not activating carrying_item and you are not activating radio then visit living
If you are not activating carrying_item and you are not activating radio then visit bedroom
If you are not activating carrying_item and you are not activating radio then visit deck
If you are not activating carrying_item and you are not activating radio then visit kitchen
If you are not activating carrying_item and you are not activating radio then visit dining
if you are activating carrying_item and you are not activating radio then visit porch

