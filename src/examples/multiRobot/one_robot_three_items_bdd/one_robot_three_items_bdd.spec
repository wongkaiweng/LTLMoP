# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
rob1_pickup, 1
rob1_drop, 1

CompileOptions:
convexify: True
parser: structured
symbolic: True
use_region_bit_encoding: False
synthesizer: slugs
fastslow: True
decompose: True

CurrentConfigName:
one_robot_three_items

Customs: # List of custom propositions
rob1_carrying_metal
rob1_carrying_glass
rob1_carrying_paper

RegionFile: # Relative path of region description file
eight_regions.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
rob1_metal, 1
rob1_glass, 1
rob1_paper, 1
rob1_drop_ac, 1
rob1_pickup_ac, 1


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
visit r1

