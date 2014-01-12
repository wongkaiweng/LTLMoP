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
flag_capture.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
damOpen, 1
eruption, 1
bFlag, 1
rFlag, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
town = p5
river = p6
Tsection = p11
others = p1, p2
grassland = p14, p15, p16, p17
volcano = p3
campground = p20, p21, p22, p23, p24, p25, p26, p27, p28
path = p7

Spec: # Specification in structured English
#infinitely often not damOpen
#infinitely often not eruption
#infinitely often bFlag and rFlag

go to town
go to volcano

if you were sensing damOpen then do not river
if you were sensing eruption then do not path

if you are not sensing bFlag and rFlag then do not Tsection

