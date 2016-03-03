# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
sit, 1

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
sense_danger

RegionFile: # Relative path of region description file
eight_regions.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
danger, 1
safe, 1


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

sense_danger is set on danger and reset on safe

do sit if and only if you were in r8 and you are activating sense_danger

if you are activating sit then stay there

if you are activating sense_danger then visit r8

if you are not activating sense_danger then visit all regions

