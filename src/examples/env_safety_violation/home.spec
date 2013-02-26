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
env_safety_violation.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
cooking, 1
sleeping, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
kitchen = p2
others = p4, p5, p6, p7, p8, p9, p10, p11
bedroom = p3

Spec: # Specification in structured English
go to kitchen
go to bedroom

do cooking  if and only if you are in kitchen
#if you are in bedroom then do sleeping

