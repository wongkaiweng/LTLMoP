# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
cooking, 1

CompileOptions:
neighbour_robot: True
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
interactive: True

CurrentConfigName:
nao_basicSim

Customs: # List of custom propositions
receivedIngredient

RegionFile: # Relative path of region description file
../workspace_new.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
ingredientArrived, 1
assistant_passIngredient, 1
assistant_ActpassIngredient, 1
assistant_company, 1
assistant_road, 1
assistant_preparationArea, 1
assistant_storage, 1
assistant_cookingArea, 1
deliveryAgent_company, 1
deliveryAgent_road, 1
deliveryAgent_preparationArea, 1
deliveryAgent_storage, 1
deliveryAgent_cookingArea, 1


======== SPECIFICATION ========

GlobalSensors: # Sensors accessible by all robots

OtherRobot: # The other robot in the same workspace
assistant
deliveryAgent

RegionMapping: # Mapping between region names and their decomposed counterparts
preparationArea = p3
company = p5
storage = p1
others = 
cookingArea = p4
road = p2

Spec: # Specification in structured English
Robot starts in cookingArea

# locations of the other robots
Env starts with assistant_preparationArea and deliveryAgent_company
always assistant_preparationArea
always not (deliveryAgent_cookingArea or deliveryAgent_preparationArea)

# always stay in cooking area
if you have finished cookingArea then do cookingArea

# the other robot passes ingredient if the ingredient arrives
if you were sensing ingredientArrived then do assistant_ActpassIngredient

# remember if you have received ingredient
receivedIngredient is set on assistant_passIngredient and reset on finished cooking

# cook only if you get the ingredient
do cooking if and only if you are sensing receivedIngredient
if you are sensing ingredientArrived or receivedIngredient then visit cooking

# assume how the other robot works
infinitely often not assistant_ActpassIngredient or assistant_passIngredient

#always (not ingredientArrived or assistant_ActpassIngredient)

