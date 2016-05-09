# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)

CompileOptions:
neighbour_robot: False
convexify: False
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
decompose: False
interactive: False

CurrentConfigName:
basicSim

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
../../../../../../../home/catherine/LTLMoP/src/examples/fmrChallenge/output_2_3.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)


======== SPECIFICATION ========

GlobalSensors: # Sensors accessible by all robots

OtherRobot: # The other robot in the same workspace

RegionMapping: # Mapping between region names and their decomposed counterparts
segment_2_bottom_lane = p5
segment_1_right_intersect = p7
segment_0_left_intersect = p11
segment_2_top_lane = p3
segment_0_right_intersect = p10
segment_3_top_lane = p1
segment_0_top_lane = p9
others = 
segment_2_right_intersect = p4
segment_3_bottom_lane = p2
segment_0_bottom_lane = p12
segment_1_bottom_lane = p8
segment_1_top_lane = p6

Spec: # Specification in structured English
Robot starts in segment_2_bottom_lane

#visit segment_0_top_lane (2_2)
visit segment_4_top_lane

