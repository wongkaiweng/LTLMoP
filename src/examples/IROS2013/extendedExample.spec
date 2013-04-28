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
extendedExample.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
Door1Closed, 1
Door2Closed, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
h8 = p5
h9 = p4
Goal = p16
r3 = p2
h2 = p11
h3 = p10
h1 = p13
h6 = p7
h7 = p6
h4 = p9
h5 = p8
h10 = p12
Door1 = p18
Door2 = p17
Start = p15
others = p1
r1 = p3

Spec: # Specification in structured English
Robot starts in Start
go to Goal

always not Door1Closed or not Door2Closed
if you were in h3 or h4 and Door1Closed then do Door1Closed
if you were in h3 or h4 and Door2Closed then do Door2Closed
if you were in h2 or h5 and Door1Closed then do Door1Closed
if you were in h2 or h5 and Door2Closed then do Door2Closed
if you were in h3 or h4 and not Door1Closed then do not Door1Closed
if you were in h3 or h4 and not Door2Closed then do not Door2Closed
if you were in h2 or h5 and not Door1Closed then do not Door1Closed
if you were in h2 or h5 and not Door2Closed then do not Door2Closed


if Door1Closed then do not Door1
if Door2Closed then do not Door2
#always not Door1 or not Door1Closed
#always not Door2 or not Door2Closed

