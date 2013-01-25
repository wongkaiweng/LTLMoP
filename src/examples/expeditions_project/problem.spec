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
Original

Customs: # List of custom propositions
OrderReceived
FoodObtained

RegionFile: # Relative path of region description file
../../../../Dropbox/NSF ExCAPE/restaurant.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
foodReady, 1
order_soup, 0
order_rice, 1
order_sake, 0


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
kitchen_sake = p2
others = p7, p8, p9, p10, p11, p12, p13, p14, p15
c1 = p5
kitchen_rice = p3
c2 = p4

Spec: # Specification in structured English
Robot starts in kitchen_sake
#group kitchen is kitchen_rice, kitchen_soup,kitchen_sake
#group orderFood is order_rice, order_soup, order_sake
# For tracking the type of food ordered


#Assumptions of the environment
# 1.
OrderReceived is set on order_rice and reset on deliver
if you were sensing start of order_rice then stay there
#any orderFood and reset on deliver
FoodObtained is set on pickup and reset on  deliver
#if you were activating OrderReceived  then do not order_rice
#(order_rice or order_soup or order_sake)
# 2.
Infinitely often foodReady
# 3.
#if you were sensing foodReady and you were not activating kitchen_rice and pickup then do foodReady
#any kitchen and pickup then do foodReady
#if you were sensing foodReady and you were activating kitchen_rice and pickup then do not foodReady
#any kitchen and pickup then do not foodReady

#Guarantee that the robot needs to fulfill
# 1. in LTLMoP
# 2.
#GoToPickup is set on deliver and reset on pickup
#If you are activating GoToPickup then do not deliver
#If you are not activating GoToPickup then do not pickup

# 3.
if you are sensing OrderReceived and foodReady and not FoodObtained then go to kitchen_rice
do pickup if and only if you are in kitchen_rice and foodReady and not FoodObtained
#all kitchen
if you are sensing OrderReceived and FoodObtained then go to c1 and deliver
#if you are sensing OrderReceived and FoodObtained and c1 then do deliver
if you are not in c1 then do not deliver
#do deliver if and only if you are in c1 and OrderReceived and FoodObtained
#Always not deliver
#FoodDelivered is set on deliver and c1 and reset on orderFood
# 4.
#if orderFood then do reachCustomer
#not reachCustomer is set on deliver and not customer1 and reset on pickup and appetizer and EndOfQueue
#not EndOfQueue is set on not deliver and reset on deliver and  customer1
# 5. ????
If you are not sensing OrderReceived then do not deliver

