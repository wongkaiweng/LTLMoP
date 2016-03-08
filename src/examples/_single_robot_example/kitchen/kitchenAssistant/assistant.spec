# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
passIngredient, 1
openDoor, 1

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
Untitled configuration

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
../workspace.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
ingredientArrived, 1
chef_receivedIngredient, 0
chef_cooking, 0
deliveryAgent_road, 1
deliveryAgent_storage, 1
deliveryAgent_orderDelivery, 0
deliveryAgent_company, 0


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
Robot starts in preparationArea

if you are activating passIngredient then stay there
if you were not sensing ingredientArrived and preparationArea then do not  passIngredient

## added by chef in NEGO
#if you were sensing ingredientArrived then do passIngredient
## --- goals ---- #
#infinitely often not( ingredientArrived or chef_receivedIngredient ) or chef_cooking

## added by deliveryAgent by NEGO
if you are sensing deliveryAgent_road or deliveryAgent_storage then do openDoor
## --- goals --- #
#infinitely often deliveryAgent_orderDelivery or deliveryAgent_company
#infinitely often not deliveryAgent_orderDelivery or deliveryAgent_storage


# ---- obselete --- #
#if you are sensing ingredientArrived then visit preparationArea
#if you are not sensing ingredientArrived then visit storage

## you can only pass ingredient in preparationArea
#if you have finished cookingArea or storage or road or company then do not passIngredient

