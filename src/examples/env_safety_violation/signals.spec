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
if you were in green then do move
if you were in red then do stop
always (move or stop)
always not (move and stop)

go to green
go to yellow
go to red

