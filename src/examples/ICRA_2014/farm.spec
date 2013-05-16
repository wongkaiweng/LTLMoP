# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)

CompileOptions:
convexify: True
fastslow: False

CurrentConfigName:
Untitled configuration

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
farm.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
sunlight, 0
irrigation_left, 1
irrigation_right, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
tomato = p2
house = p4
corn = p6
cucumber = p5
others = 
basil = p7
road = p3

Spec: # Specification in structured English

# env assumptions (mode 1)
always not (irrigation_left and irrigation_right)
infinitely often not irrigation_left
infinitely often irrigation_left

go to road
go to house

#if you are sensing sunlight then do not corn and cucumber and tomato and corn
if you are sensing irrigation_left then do not tomato and not basil
if you are sensing irrigation_right then do not corn and not cucumber

