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
flag, 1
dig, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
others = p4, p5, p6, p7, p8, p9, p10, p11
r2 = p3
r3 = p2

Spec: # Specification in structured English
go to r2
go to r3

do flag if and only if you are in r2
do dig if and only if you are in r3

