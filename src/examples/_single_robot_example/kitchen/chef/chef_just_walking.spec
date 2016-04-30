# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
cooking, 1

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
recovery: False
include_heading: False
winning_livenesses: False
synthesizer: slugs
decompose: True
interactive: False

CurrentConfigName:
nao_basicSim

Customs: # List of custom propositions
receivedIngredient

RegionFile: # Relative path of region description file
../../../../../../../../../home/catherine/LTLMoP/src/examples/_single_robot_example/kitchen/workspace_new.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
ingredientArrived, 1
assistant_passIngredient, 1
assistant_ActpassIngredient, 1


======== SPECIFICATION ========

GlobalSensors: # Sensors accessible by all robots

OtherRobot: # The other robot in the same workspace

RegionMapping: # Mapping between region names and their decomposed counterparts
preparationArea = p3
company = p5
storage = p1
others = 
cookingArea = p4
road = p2

Spec: # Specification in structured English
Robot starts in company

visit company
visit storage

