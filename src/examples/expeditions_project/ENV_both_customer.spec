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
BasicSim

Customs: # List of custom propositions
FoodObtained
Orderc1c2
SakeOrdered
RiceOrdered

RegionFile: # Relative path of region description file
../../../../Dropbox/NSF ExCAPE/restaurant.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
order_rice, 1
order_sake, 1
rice_ready, 1
sake_ready, 1
c1_order, 1
c2_order, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
kitchen_sake = p2
others = p7, p8, p9, p10, p11, p12, p13, p14, p15
c1 = p5
kitchen_rice = p3
c2 = p4

Spec: # Specification in structured English
Robot starts in kitchen_sake

###### ENVIRONMENT ASSUMPTIONS ######
# Environment Liveness Assumptions
if you are activating RiceOrdered and not FoodObtained then infinitely often rice_ready
if you are activating SakeOrdered and not FoodObtained then infinitely often sake_ready

# Environment Safety Assumptions
if you were activating c1_order then do not c2_order
if you were activating c2_order then do not c1_order

###### FOOD ORDERING ######
## Tracking the type of food ordered ##
RiceOrdered is set on order_rice and reset on deliver
SakeOrdered is set on order_sake and reset on deliver
if you were sensing start of RiceOrdered then stay there
if you were sensing start of SakeOrdered then stay there

## Tracking which customer ordered food ##
Orderc1c2 is set on c1_order and reset on c2_order
if you were sensing start of c1_order then stay there

###### FOOD RECEIVING ######
if you are sensing RiceOrdered and rice_ready and not FoodObtained then go to kitchen_rice
if you are sensing SakeOrdered and sake_ready and not FoodObtained then go to kitchen_sake
do pickup if and only if you are in kitchen_rice and rice_ready and RiceOrdered or you are in kitchen_sake and sake_ready and SakeOrdered
FoodObtained is set on pickup and reset on deliver

###### FOOD DELIVERING ######
if you are sensing (SakeOrdered or RiceOrdered)  and FoodObtained and Orderc1c2 then go to c1 and deliver
if you are sensing (SakeOrdered  or RiceOrdered) and FoodObtained and not Orderc1c2 then go to c2 and deliver

if you are sensing (SakeOrdered or RiceOrdered) and FoodObtained and Orderc1c2 and c1 then do deliver
if you are sensing (SakeOrdered or RiceOrdered) and FoodObtained and not Orderc1c2 and c2 then do deliver

if you are not in (c1 and FoodObtained and Orderc1c2) or (c2 and FoodObtained and not Orderc1c2) then do not deliver

If you were not sensing SakeOrdered or RiceOrdered then do not deliver

