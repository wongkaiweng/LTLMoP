# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
extinguishFire, 1

CompileOptions:
convexify: True
fastslow: False

CurrentConfigName:
Untitled configuration

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
camping.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
earthquake, 0
thief, 1
fire, 1
entranceClosed, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
entrance = p6
office = p4
others = 
park = p3
shelter1 = p1
Room3 = p8
corridor = p7
Room1 = p10
home = p5
Room2 = p9
serverRoom = p2

Spec: # Specification in structured English

group allRegion is Room1,Room2, Room3, serverRoom

# adding liveness will work
always not entrance or not entranceClosed
infinitely often not entranceClosed

if you are not sensing thief then visit all allRegion
do extinguishFire if and only if you are sensing fire
#if you are sensing fire then do extinguishFire
if you are sensing thief then go to office and stay there

always not (extinguishFire and serverRoom)


######with earthquake
#####if you are not sensing (earthquake or thief) then visit all allRegion
#####if you are sensing earthquake and not thief then go to shelter1 and stay there
#######always not (thief or fire or earthquake)

# Some assumption that make the specification realizable
#always not (thief or fire)
#always not (fire)
#always thief or not fire

