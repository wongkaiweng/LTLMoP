# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
pickup, 1
drop, 1

CompileOptions:
convexify: True
parser: structured
symbolic: False
use_region_bit_encoding: True
synthesizer: jtlv
fastslow: False
decompose: True

CurrentConfigName:
nao

Customs: # List of custom propositions
gotMetal
gotGlass
gotPaper

RegionFile: # Relative path of region description file
eight_regions.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
metal, 1
glass, 1
paper, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
r4 = p5
r5 = p4
r6 = p3
r7 = p2
r1 = p8
r2 = p7
r3 = p6
r8 = p1
others = 

Spec: # Specification in structured English
group regions is r1,r2,r4,r5,r7
robot starts with false

# pickup object if you see them
do pickup if and only if you are sensing metal or glass or paper

# remember what you have picked up
gotMetal is set on metal and pickup and reset on r6 and drop
gotGlass is set on glass and pickup and reset on r8 and drop
gotPaper is set on paper and pickup and reset on r3 and drop

# drop the object at the right place
do drop if and only if you are sensing (gotMetal and r6) or (gotGlass and r8) or (gotPaper and r3)
#if you are activating start of drop then stay there
#do drop if and only if you are sensing (r6 or r8 or r3)

if you are not activating (metal or glass or paper) then visit all regions
if you are activating gotMetal then visit r6
if you are activating gotGlass then visit r8
if you are activating gotPaper then visit r3

