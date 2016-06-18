# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
rob1_pickup, 1
rob1_drop, 1

CompileOptions:
neighbour_robot: False
convexify: True
parser: structured
symbolic: False
use_region_bit_encoding: True
multi_robot_mode: negotiation
cooperative_gr1: True
fastslow: True
only_realizability: False
recovery: False
include_heading: False
winning_livenesses: False
synthesizer: slugs
decompose: True
interactive: True

CurrentConfigName:
basicSim

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


======== SPECIFICATION ========

GlobalSensors: # Sensors accessible by all robots

OtherRobot: # The other robot in the same workspace

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

# continue to see objects until pickup is finished
if you were sensing rob1_metal and not finished rob1_pickup then do rob1_metal
if you were sensing rob1_glass and you have not finished rob1_pickup then do rob1_glass
if you were sensing rob1_paper and you have not finished rob1_pickup then do rob1_paper

# carry_metal
if you are sensing rob1_metal and you were not sensing rob1_carrying_metal or rob1_carrying_glass or rob1_carrying_paper then do rob1_pickup
rob1_carrying_metal is set on rob1_metal and finished rob1_pickup and reset on finished r6 and finished rob1_drop
#do rob1_carrying_metal if and only if you are sensing rob1_metal and finished rob1_pickup or you were sensing  rob1_carrying_metal and you have not finished r6 and rob1_drop

# carry_glass
if you are sensing rob1_glass and not rob1_carrying_metal and not rob1_carrying_glass and not rob1_carrying_paper then do rob1_pickup
rob1_carrying_glass is set on rob1_glass and finished rob1_pickup and reset on finished r8 and finished rob1_drop
#do rob1_carrying_glass if and only if you were sensing rob1_glass and finished rob1_pickup or you were sensing  rob1_carrying_glass and not finished r8 and  not finished rob1_drop

# carry_paper
if you are sensing rob1_paper and not rob1_carrying_metal and not rob1_carrying_glass and not rob1_carrying_paper then do rob1_pickup
rob1_carrying_paper is set on rob1_paper and finished rob1_pickup and reset on finished r3 and finished rob1_drop
#do rob1_carrying_paper if and only if you were sensing rob1_paper and finished rob1_pickup or you were sensing  rob1_carrying_paper and not finished r3 and  not finished rob1_drop

# drop at only designated regions
do rob1_drop if and only if you are sensing (rob1_carrying_metal and finished r6) or (rob1_carrying_glass and finished r8) or (rob1_carrying_paper and finished r3)

# pickup only when objects are detected
if you are not sensing (rob1_metal or rob1_glass or rob1_paper) then do not rob1_pickup

##############
### GOALS   ####
##############
# recycle items
if you are activating rob1_carrying_metal then visit r6
if you are activating rob1_carrying_glass then visit r8
if you are activating rob1_carrying_paper then visit r3
#if you are activating rob1_carrying_metal and finished r6 then do rob1_drop
#if you are activating rob1_carrying_glass and finished r8 then do rob1_drop
#if you are activating rob1_carrying_paper and finished r3 then do rob1_drop
if you are activating (rob1_carrying_paper and finished r3) or (rob1_carrying_metal and finished r6) or (rob1_carrying_glass and finished r8) then do rob1_drop


# patrol
if you are not activating rob1_carrying_metal or rob1_carrying_glass or rob1_carrying_paper and you are not activating rob1_pickup then visit r1
if you are not activating rob1_carrying_metal or rob1_carrying_glass or rob1_carrying_paper and you are not activating rob1_pickup then visit r2
if you are not activating rob1_carrying_metal or rob1_carrying_glass or rob1_carrying_paper and you are not activating rob1_pickup then visit r3
if you are not activating rob1_carrying_metal or rob1_carrying_glass or rob1_carrying_paper and you are not activating rob1_pickup then visit r4
if you are not activating rob1_carrying_metal or rob1_carrying_glass or rob1_carrying_paper and you are not activating rob1_pickup then visit r5
if you are not activating rob1_carrying_metal or rob1_carrying_glass or rob1_carrying_paper and you are not activating rob1_pickup then visit r6
if you are not activating rob1_carrying_metal or rob1_carrying_glass or rob1_carrying_paper and you are not activating rob1_pickup then visit r7
if you are not activating rob1_carrying_metal or rob1_carrying_glass or rob1_carrying_paper and you are not activating rob1_pickup then visit r8

