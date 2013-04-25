# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
drop_ball, 1

CompileOptions:
convexify: True
fastslow: False

CurrentConfigName:
tutorial

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
tutorial.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
person, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
r4 = p1
others = 
r1 = p4
r2 = p3
r3 = p2

Spec: # Specification in structured English
go to r1
go to r2
go to r3
go to r4
if you were sensing person then do drop_ball

