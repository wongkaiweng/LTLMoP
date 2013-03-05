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
classroom = p16
h8 = p6
h9 = p5
r1 = p4
r2 = p3
h2 = p12
h3 = p11
h1 = p14
h6 = p8
h7 = p7
h4 = p10
h5 = p9
h10 = p13
Door1 = p19
Door2 = p18
others = p1, p2
Goal = p17

Spec: # Specification in structured English
Robot starts in classroom
go to Goal

always not Door1Closed or not Door2Closed
if you were in (h3 or h4 or r2) and Door1Closed then do Door1Closed
if you were in (h3 or h4 or r2) and Door2Closed then do Door2Closed
if you were in (h2 or h5 or r1) and Door1Closed then do Door1Closed
if you were in (h2 or h5 or r1) and Door2Closed then do Door2Closed
if you were in (h3 or h4 or r2) and not Door1Closed then do not Door1Closed
if you were in (h3 or h4 or r2) and not Door2Closed then do not Door2Closed
if you were in (h2 or h5 or r1) and not Door1Closed then do not Door1Closed
if you were in (h2 or h5 or r1) and not Door2Closed then do not Door2Closed


if Door1Closed then do not Door1
if Door2Closed then do not Door2
#always not Door1 or not Door1Closed
#always not Door2 or not Door2Closed

