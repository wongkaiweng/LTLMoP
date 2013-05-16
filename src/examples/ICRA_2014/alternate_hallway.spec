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
alternate_hallway.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
dog, 1
cat, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
porch = p1
hall_cat = p3
bedroom = p5
others = 
hall_dog = p2
dining_room = p4

Spec: # Specification in structured English
#infinitely often not (dog and cat)

# Case 1 (good for mode 1 and mode 3)
#if you were in porch then do not cat
#if you were in dining_room then do (not dog or not cat)

# Case 2 (mode 2)
#if you were sensing dog then do not cat

#safety guarantees
go to porch
go to bedroom
if you are sensing dog then do not hall_dog
if you are sensing cat then do not hall_cat

