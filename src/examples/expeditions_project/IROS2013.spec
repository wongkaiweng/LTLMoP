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
basic

Customs: # List of custom propositions
FoodObtained
c1Ordered
c2Ordered

RegionFile: # Relative path of region description file
two_customer_res.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
c1_order, 1
c2_order, 1
foodReady, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
c2 = p4
c1 = p5
kitchen = p3
kitchen_sake = p2
others = p7, p8, p9, p10, p11, p12, p13, p14, p15, p16, p17

Spec: # Specification in structured English
#Assumptions of the environment
if you are activating (c1Ordered or c2Ordered) and not FoodObtained then infinitely often foodReady

###### FOOD ORDERING ######
## Tracking which customer ordered food ##
c1Ordered is set on c1_order and reset on deliver
c2Ordered is set on c2_order and reset on deliver
if you were sensing start of c1Ordered then stay there
if you were sensing start of c2Ordered then stay there

###### FOOD RECEIVING ######
if you are sensing (c1Ordered or c2Ordered) and foodReady and not FoodObtained then go to kitchen
do pickup if and only if you are in kitchen and foodReady and (c1Ordered or c2Ordered)
FoodObtained is set on pickup and reset on deliver

###### FOOD DELIVERING ######
if you are sensing c1Ordered  and FoodObtained then go to c1 and deliver
if you are sensing c2Ordered and FoodObtained then go to c2 and deliver

if you are not in (c1 and FoodObtained and c1Ordered) or (c2 and FoodObtained and c2Ordered) then do not deliver

If you were not sensing (c1Ordered or c2Ordered) then do not deliver

