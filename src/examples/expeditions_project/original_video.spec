# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
pickup, 1
deliver, 1

CompileOptions:
convexify: True
fastslow: False

CurrentConfigName:
Nao

Customs: # List of custom propositions
OrderReceived
FoodObtained

RegionFile: # Relative path of region description file
original_res.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
foodReady, 1
order_rice, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
customer = p3
kitchen_rice = p2
others = p5, p6, p7, p8, p9, p10, p11

Spec: # Specification in structured English
Robot starts in kitchen_rice

#Assumptions of the environment
Infinitely often foodReady

## Food Ordering ##
OrderReceived is set on order_rice and reset on deliver
if you were sensing start of order_rice then stay there

## Food Receiving ##
if you are sensing OrderReceived and foodReady and not FoodObtained then go to kitchen_rice
do pickup if and only if you are in kitchen_rice and OrderReceived and foodReady and not FoodObtained
FoodObtained is set on pickup and reset on  deliver

## Food Delivering ##
if you are sensing OrderReceived and FoodObtained then go to customer and deliver
if you are sensing OrderReceived and FoodObtained and customer then do deliver
if you are not in customer and FoodObtained then do not deliver
if you are not sensing OrderReceived then do not deliver

