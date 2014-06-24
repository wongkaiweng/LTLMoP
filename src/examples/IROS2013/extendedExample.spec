# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)

CompileOptions:
convexify: False
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
Classroom = p9
Lbottom = p6
Rbottom = p3
Ltop = p5
others = 
Rtop = p2
Door1 = p8
Playground = p4
Door2 = p7

Spec: # Specification in structured English
Robot starts in Classroom
go to Playground

#always not Door1Closed or not Door2Closed
#if you were in Ltop and Door1Closed then do Door1Closed
#if you were in Ltop and Door2Closed then do Door2Closed
#if you were in Rtop and Door1Closed then do Door1Closed
#if you were in Rtop and Door2Closed then do Door2Closed
#if you were in Ltop and not Door1Closed then do not Door1Closed
#if you were in Ltop and not Door2Closed then do not Door2Closed
#if you were in Rtop and not Door1Closed then do not Door1Closed
#if you were in Rtop and not Door2Closed then do not Door2Closed


if you are sensing Door1Closed then do not Door1
if you are sensing Door2Closed then do not Door2
#always not Door1 or not Door1Closed
#always not Door2 or not Door2Closed

