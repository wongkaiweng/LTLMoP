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
doNotOrder
EndOfQueue
reachCustomer

RegionFile: # Relative path of region description file
original.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
orderFood, 1
foodReady, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
guy = p2
others = p1
girl = p3
appetizer = p4

Spec: # Specification in structured English
#if windy and wet then do cold

Robot starts in appetizer
# change to the group later?
group customer is girl,guy
#Assumptions of the environment
# 1.
if orderFood then do deliver
#do toCustomer if and only if deliver and you are in customer
#Do not orderFood unless you are in customer

# 2.
Infinitely often foodReady
# 3.
#if foodReady then do not foodReady
#do not foodReady if and only if you were in appetizer and pickup


#Guarantee that the robot needs to fulfill
# 1.
# 2.
#if deliver then do cold

# 3.
# 4.
not reachCustomer is set on not deliver and reset on pickup and appetizer and EndOfQueue
not EndOfQueue is set on not deliver and reset on deliver and  customer
# 5.

