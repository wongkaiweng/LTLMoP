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
sensors.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
sen1, 1
sen2, 1
sen3, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
start = p1
others = 
r1 = p4
r2 = p3
r3 = p2

Spec: # Specification in structured English



if you are not sensing sen1 or sen2 or sen3 then go to start

