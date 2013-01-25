# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
pickup, 1
deliver, 1

CompileOptions:
convexify: False
fastslow: False

CurrentConfigName:
Original

Customs: # List of custom propositions
OrderReceived
GoToPickup
FoodObtained
FoodDelivered

RegionFile: # Relative path of region description file
original_res.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
foodReady, 1
<<<<<<< HEAD
order_soup, 1
order_rice, 1
order_sake, 1
=======
order_rice, 1
>>>>>>> modified the spec with two customers and names it two_customer.spec


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
<<<<<<< HEAD
kitchen_soup = p2
others = p1
kitchen_sake = p3
c2 = p5
c1 = p6
kitchen_rice = p4

Spec: # Specification in structured English
Robot starts in kitchen_rice
group kitchen is kitchen_rice, kitchen_soup,kitchen_sake
group orderFood is order_rice, order_soup, order_sake

#Assumptions of the environment
# 1.
OrderReceived is set on any orderFood and reset on deliver
FoodObtained is set on pickup and reset on  deliver
if you were activating OrderReceived  then do not (order_rice or order_soup or order_sake)
# 2.
Infinitely often foodReady
# 3.
if you were sensing foodReady and you were not activating any kitchen and pickup then do foodReady
#kitchen_rice and pickup then do foodReady
if you were sensing foodReady and you were activating any kitchen and pickup then do not foodReady
#kitchen_rice and pickup then do not foodReady
=======
customer = p3
kitchen_rice = p2
others = p5, p6, p7, p8, p9, p10, p11

Spec: # Specification in structured English
Robot starts in kitchen_rice

#Assumptions of the environment
Infinitely often foodReady
#if you were activating OrderReceived then do not order_rice
#if you were sensing foodReady and you were not activating kitchen_rice and pickup then do foodReady
#if you were sensing foodReady and you were activating kitchen_rice and pickup then do not foodReady
>>>>>>> modified the spec with two customers and names it two_customer.spec

## Food Ordering ##
OrderReceived is set on order_rice and reset on deliver
if you were sensing start of order_rice then stay there

<<<<<<< HEAD
# 3.
if you are sensing OrderReceived and not FoodObtained then go to all kitchen
#kitchen_rice and pickup
if you are sensing OrderReceived and FoodObtained then go to c1
do deliver if and only if you were in c1 and OrderReceived and FoodObtained
#FoodDelivered is set on deliver and c1 and reset on orderFood
# 4.
#if orderFood then do reachCustomer
#not reachCustomer is set on deliver and not customer1 and reset on pickup and appetizer and EndOfQueue
#not EndOfQueue is set on not deliver and reset on deliver and  customer1
# 5. ????
If you are not sensing OrderReceived then do not deliver
=======
## Food Receiving ##
if you are sensing OrderReceived and foodReady and not FoodObtained then go to kitchen_rice
do pickup if and only if you are in kitchen_rice and OrderReceived and foodReady and not FoodObtained
FoodObtained is set on pickup and reset on  deliver

## Food Delivering ##
if you are sensing OrderReceived and FoodObtained then go to customer and deliver
if you are sensing OrderReceived and FoodObtained and customer then do deliver
if you are not in customer and FoodObtained then do not deliver
#do deliver if and only if you are in c1 and OrderReceived and FoodObtained
if you are not sensing OrderReceived then do not deliver
>>>>>>> modified the spec with two customers and names it two_customer.spec

