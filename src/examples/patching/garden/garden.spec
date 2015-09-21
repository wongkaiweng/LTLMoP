# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)

CompileOptions:
synthesizer: slugs
neighbour_robot: True
fastslow: True
include_heading: False
convexify: False
recovery: False
parser: structured
symbolic: False
decompose: True
use_region_bit_encoding: True

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
garden.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
bob_home, 1
bob_hotel, 1
bob_office, 1
bob_conventionCenter, 1
bob_rightBottom, 1
bob_leftBottom, 1
bob_rightTop, 1
bob_leftTop, 1
bob_right, 1
bob_bottom, 1
bob_left, 1
bob_top, 1


======== SPECIFICATION ========

OtherRobot: # The other robot in the same workspace
bob

RegionMapping: # Mapping between region names and their decomposed counterparts
rightBottom = p3
leftBottom = p7
right = p4
office = p5
bottom = p13
top = p1
hotel = p9
others = 
rightTop = p2
leftTop = p6
home = p10
conventionCenter = p12
left = p8

Spec: # Specification in structured English
robot starts in home


go to office

