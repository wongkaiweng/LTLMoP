# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
pickup, 1
deliver, 1

CompileOptions:
convexify: True
fastslow: False

Customs: # List of custom propositions
OrderReceived
FoodObtained
OrderSake_Rice
OrderSakeSoup_
RiceForPickup
SoupForPickup
SakeForPickup

RegionFile: # Relative path of region description file
../../../Dropbox/NSF ExCAPE/restaurant.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
order_soup, 1
order_rice, 1
order_sake, 1
soup_ready, 1
rice_ready, 1
sake_ready, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
kitchen_soup = p2
others = p8, p9, p10, p11, p12, p13, p14, p15
kitchen_sake = p3
c2 = p5
c1 = p6
kitchen_rice = p4

Spec: # Specification in structured English
Robot starts in kitchen_sake
group kitchen is kitchen_rice, kitchen_soup,kitchen_sake
group orderFood is order_rice, order_soup, order_sake
group foodReady is rice_ready,soup_ready,sake_ready
# For tracking the type of food ordered
# Rice: 01 , Soup: 10, Sake 11
OrderSake_Rice is set on order_sake or order_rice and reset on deliver
OrderSakeSoup_ is set on order_sake or order_soup and reset on deliver

#Assumptions of the environment
# 1.
OrderReceived is set on any orderFood and reset on deliver
#FoodObtained is set on pickup and foodReady and reset on  deliver
FoodObtained is set on pickup and reset on  deliver
#if you were activating OrderReceived  then do not (order_rice or order_soup or order_sake)
# 2.
#Infinitely often any foodReady
# 3.
#if you were sensing foodReady and you were not activating any kitchen and pickup then do foodReady
#if you were sensing foodReady and you were activating any kitchen and pickup then do not foodReady

#Guarantee that the robot needs to fulfill
# 1. in LTLMoP
# 2.
#GoToPickup is set on deliver and reset on pickup
#If you are activating GoToPickup then do not deliver
#If you are not activating GoToPickup then do not pickup

# 3.
#if you are sensing OrderReceived and foodReady and not FoodObtained then go to all kitchen
if you are sensing OrderReceived and not FoodObtained then go to all kitchen
RiceForPickup is set on kitchen_rice and not OrderSakeSoup_ and  OrderSake_Rice and reset on deliver
SoupForPickup is set on kitchen_soup and OrderSakeSoup_ and  not OrderSake_Rice and reset on deliver
SakeForPickup is set on kitchen_sake and OrderSakeSoup_ and  OrderSake_Rice and reset on deliver
do pickup if and only if RiceForPickup or SoupForPickup or SakeForPickup
#all kitchen
if you are sensing OrderReceived and FoodObtained then go to c1
do deliver if and only if you were in c1 and OrderReceived and FoodObtained
#FoodDelivered is set on deliver and c1 and reset on orderFood
# 4.
#if orderFood then do reachCustomer
#not reachCustomer is set on deliver and not customer1 and reset on pickup and appetizer and EndOfQueue
#not EndOfQueue is set on not deliver and reset on deliver and  customer1
# 5. ????
If you are not sensing OrderReceived then do not deliver

