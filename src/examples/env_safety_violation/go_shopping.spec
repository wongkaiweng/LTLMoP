# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)

CompileOptions:
convexify: True
fastslow: False

CurrentConfigName:
only BasicSim

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
go_shopping.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
hungry, 1
bored, 1
fire, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
aisle = p6
parking_lot = p4
emergency_exit = p5
restaurant = p3
others = 
shop1 = p2
shop2 = p1

Spec: # Specification in structured English

if you were in emergency_exit or parking_lot then do not hungry
if you were in emergency_exit or restaurant then do not bored

if you are not sensing fire or hungry or bored then go to shop1
if you are not sensing fire or hungry or bored then go to shop2

go to emergency_exit if and only if you are sensing fire
if you are sensing hungry then go to restaurant
if you are sensing bored then go to parking_lot

