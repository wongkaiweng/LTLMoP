# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)

CompileOptions:
convexify: True
fastslow: False

CurrentConfigName:
BasicSim

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
simpleExample.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
closedDoor, 1
light, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
door = p5
room1 = p4
room5 = p1
room2 = p3
room3 = p2
others = 

Spec: # Specification in structured English
#Robot starts in room1

# Env assumptions
#always not closedDoor
#infinitely often not closedDoor

# System guarantees
go to room5
if you are sensing light then do not room3
always not door or not closedDoor

