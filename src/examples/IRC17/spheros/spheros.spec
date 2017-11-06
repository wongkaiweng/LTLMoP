# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)

CompileOptions:
neighbour_robot: False
convexify: True
parser: structured
symbolic: False
use_region_bit_encoding: True
multi_robot_mode: negotiation
cooperative_gr1: True
fastslow: False
only_realizability: True
recovery: False
include_heading: False
winning_livenesses: False
synthesizer: slugs
decompose: True
interactive: True

CurrentConfigName:
Untitled configuration

Customs: # List of custom propositions
move_left
move_right
move_up
move_down
stop
turn_red
turn_yellow
turn_green
turn_blue
turn_purple
turn_white

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
left_arrow, 1
right_arrow, 1
up_arrow, 1
down_arrow, 1
stop_sign, 1


======== SPECIFICATION ========

GlobalSensors: # Sensors accessible by all robots

OtherRobot: # The other robot in the same workspace

Spec: # Specification in structured English
if you are sensing left_arrow then do move_left and turn_purple
if you are sensing right_arrow then do move_right and turn_blue
if you are sensing up_arrow then do move_up and turn_green
if you are sensing down_arrow then do move_down and turn_yellow
if you are sensing stop_sign then do stop and turn_red

if you are not sensing left_arrow or right_arrow or up_arrow or down_arrow or stop_sign then do turn_white

