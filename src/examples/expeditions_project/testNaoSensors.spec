# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
receive_order_rice, 1

CompileOptions:
convexify: True
fastslow: False

CurrentConfigName:
testNaoSensors

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
testNaoSensors.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
order_rice, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
others = 
r1 = p3
r2 = p2
r3 = p1

Spec: # Specification in structured English
visit r1
visit r2
visit r3

if you are sensing start of order_rice then do receive_order_rice

