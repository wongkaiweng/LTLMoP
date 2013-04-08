# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)

CompileOptions:
convexify: True
fastslow: False

CurrentConfigName:
basicSim

Customs: # List of custom propositions
BlockedAmem

RegionFile: # Relative path of region description file
patrol.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
BlockedA, 1
BlockedB, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
r4 = p6
r5 = p5
r6 = p4
r7 = p3
r1 = p10
r2 = p9
r3 = p8
r8 = p2
others = 

Spec: # Specification in structured English
go to r1
go to r2
go to r5
go to r7

# Robot starts with false
#Do BlockedAmem if you are sensing BlockedA
#Do BlockedAmem if you were activating BlockedAmem

# BlockedAmem is set on BlockedA and reset on false

always not BlockedA or not BlockedB
do BlockedA unless you were sensing not BlockedA
# do BlockedA unless you sensed not BlockedAmem
always not r3 or not BlockedA
always not r6 or not BlockedB

