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
cold
atKitchen
doNotOrder
food_3
food_3_1
food_3_2

RegionFile: # Relative path of region description file
original.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
orderFood, 1
foodReady, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
customer = p2
others = p1
appetizer = p3

Spec: # Specification in structured English
#if windy and wet then do cold

Robot starts in appetizer
# change to the group later?

#Assumptions of the environment
# 1.
if orderFood then do doNotOrder
# 2.
Infinitely often foodReady
# 3.
do atKitchen if and only if you were in appetizer and you were activating pickup
do food_3_1 if and only if you were activating atKitchen and you are not sensing foodReady
do food_3_2 if and only if you were not activating atKitchen and you are sensing foodReady
if you are in appetizer then do foodReady
#if (you were activating atKitchen and you are not sensing foodReady )or (you were not activating atKitchen and you are sensing foodReady ) then do food_3

#(you were activating atKitchen and you are not sensing foodReady )or (you were not activating atKitchen and you are sensing foodReady)
#do foodReady if and only if you were not sensing atKitchen
#if you were sensing foodReady then do cold
#if you were activating food_3  then do cold

do food_3 if and only if you are activating food_3_1 or food_3_2
#if you are sensing foodReady then do food_3

#Guarantee that the robot needs to fulfill
# 1.
# 2.
#if deliver then do cold

# 3.
# 4.
# 5.

