# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
passIngredient, 1
openDoor, 1

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

RegionFile: # Relative path of region description file
../workspace_new.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
ingredientArrived, 1
chef_receivedIngredient, 1
deliveryAgent_road, 1
deliveryAgent_storage, 1
deliveryAgent_orderDelivery, 1
deliveryAgent_company, 1
chef_company, 1
chef_road, 1
chef_preparationArea, 1
chef_storage, 1
chef_cookingArea, 1
chef_Actcooking, 1
chef_cooking, 1
deliveryAgent_preparationArea, 1
deliveryAgent_cookingArea, 1


======== SPECIFICATION ========

GlobalSensors: # Sensors accessible by all robots

OtherRobot: # The other robot in the same workspace
chef
deliveryAgent

RegionMapping: # Mapping between region names and their decomposed counterparts
preparationArea = p3
company = p5
storage = p1
others = 
cookingArea = p4
road = p2

Spec: # Specification in structured English
Robot starts in preparationArea

# locations of the other robots
Env starts with chef_cookingArea and deliveryAgent_company
always chef_cookingArea
always not (deliveryAgent_cookingArea or deliveryAgent_preparationArea)

# assumptions about the passIngredient action
if you are activating passIngredient then stay there
if you were not sensing ingredientArrived and preparationArea then do not  passIngredient

# stay in place
if you have finished preparationArea then do preparationArea

## added by chef in NEGO
#if you were sensing ingredientArrived then do passIngredient
## --- goals ---- #
#infinitely often not( ingredientArrived or chef_receivedIngredient ) or chef_Actcooking

## added by deliveryAgent by NEGO
#if you are sensing deliveryAgent_road or deliveryAgent_storage then do openDoor
## --- goals --- #
#infinitely often deliveryAgent_orderDelivery or deliveryAgent_company
#infinitely often not deliveryAgent_orderDelivery or deliveryAgent_storage


# ---- obselete --- #
#if you are sensing ingredientArrived then visit preparationArea
#if you are not sensing ingredientArrived then visit storage

## you can only pass ingredient in preparationArea
#if you have finished cookingArea or storage or road or company then do not passIngredient

