# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
button_A, 1
button_B, 1
button_C, 1

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
interactive: False

CurrentConfigName:
Untitled configuration

Customs: # List of custom propositions
allLightsOn

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
light_2, 1
light_1, 1
light_3, 1


======== SPECIFICATION ========

GlobalSensors: # Sensors accessible by all robots

OtherRobot: # The other robot in the same workspace

Spec: # Specification in structured English
Env starts with not light_1 and not light_2 and not light_3
#Robot starts with false

infinitely often not button_A or finished button_A
infinitely often not button_B or finished button_B
infinitely often not button_C or finished button_C

#visit allLightsOn and light_1 and light_2 and light_3
visit light_1 and light_2 and light_3

# assumptions
#do light_1 if and only if you have finished button_A
#do light_2 if and only if you have finished button_B
#do light_3 if and only if you have finished button_C

