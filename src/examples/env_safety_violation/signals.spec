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
signals.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
stop, 1
move, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
green = p3
red = p2
yellow = p1
others = 

Spec: # Specification in structured English

do move if and only if you were in green
do stop if and only if you were in red

go to green
go to yellow
go to red

