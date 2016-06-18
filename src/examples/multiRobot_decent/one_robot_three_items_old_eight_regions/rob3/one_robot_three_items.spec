# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
rob3_pickup, 1
rob3_drop, 1

CompileOptions:
neighbour_robot: False
recovery: False
convexify: True
include_heading: False
parser: structured
symbolic: False
winning_livenesses: False
use_region_bit_encoding: True
multi_robot_mode: False
synthesizer: slugs
cooperative_gr1: True
fastslow: True
only_realizability: False
decompose: True
interactive: False

CurrentConfigName:
basicSim_MATLAB

Customs: # List of custom propositions
rob3_carrying_metal
rob3_carrying_glass
rob3_carrying_paper
all_done
r2_done
r4_done
r6_done
r8_done

RegionFile: # Relative path of region description file
../../eight_regions_old.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
rob3_metal, 1
rob3_glass, 1
rob3_paper, 1
rob3_pickup_ac, 1
other_in_r2, 1
other_in_r4, 1
other_in_r6, 1
other_in_r8, 1


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
Robot starts in r1

