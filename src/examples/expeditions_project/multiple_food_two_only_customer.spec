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
OrderSake_
Order_Rice
Order_c1

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
group kitchen is kitchen_rice, kitchen_sake
group orderFood is order_rice, order_sake
group foodReady is rice_ready,sake_ready
# For tracking the type of food ordered
# Rice: 01 ,  Sake: 10
Order_Rice is set on order_rice and reset on deliver
OrderSake_ is set on order_sake and reset on deliver
# For tracking which customer ordered food
# c1: 01 ,  c2: 10
Order_c1 is set on c1_order and reset on c2_order
#Orderc2_ is set on c2_order and reset on deliver

#Assumptions of the environment
# 1.
OrderReceived is set on any orderFood and reset on deliver
# maybe add foodReady for FoodObtained
FoodObtained is set on pickup and reset on  deliver
#if you were activating OrderReceived  then do not (order_rice or order_sake)
# 2.
Infinitely often rice_ready or sake_ready
# 3.
#if you were sensing (order_rice or order_sake) and you were not activating any kitchen and pickup then do (order_rice or order_sake)
#if you were sensing order_rice  and you were activating kitchen_rice and pickup then do not order_rice
#if you were sensing order_sake  and you were activating kitchen_sake and pickup then do not order_sake

#Guarantee that the robot needs to fulfill
# 1. in LTLMoP  # 2. implied either deliver or pickup
# 3.
#maybe OrderReceived and foodReady and not FoodObtained
if you are sensing not OrderSake_ and Order_Rice and rice_ready  and not FoodObtained then go to kitchen_rice
if you are sensing sake_ready and OrderSake_ and not Order_Rice and not FoodObtained then go to kitchen_sake

do pickup if and only if you are in kitchen_rice and rice_ready and not OrderSake_ and Order_Rice or you are in kitchen_sake and sake_ready and OrderSake_ and not Order_Rice

if you are sensing OrderReceived and FoodObtained and Order_c1 then go to c1
if you are sensing OrderReceived and FoodObtained and not Order_c1 then go to c2
do deliver if and only if you were in c1 and Order_c1 and OrderReceived and FoodObtained or you were in c2 and not Order_c1 and OrderReceived and FoodObtained



# 4.
#if orderFood then do reachCustomer
#not reachCustomer is set on deliver and not customer1 and reset on pickup and appetizer and EndOfQueue
#not EndOfQueue is set on not deliver and reset on deliver and  customer1
# 5. ????
If you are not sensing OrderReceived then do not deliver

