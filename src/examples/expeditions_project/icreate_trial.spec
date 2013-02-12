# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
pickup, 1
deliver, 1

CompileOptions:
convexify: True
fastslow: False

CurrentConfigName:
iCreate

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
two_customer_res.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
order_rice, 0
order_sake, 0
rice_ready, 0
sake_ready, 0
c1_order, 1
c2_order, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
others = p7, p8, p9, p10, p11, p12, p13, p14, p15, p16, p17
kitchen_sake = p2
c2 = p4
c1 = p5
kitchen_rice = p3

Spec: # Specification in structured English
go to c1
go to kitchen_rice

if you are sensing c2_order then do pickup
if you are sensing c1_order then do deliver

