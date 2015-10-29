# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)

CompileOptions:
neighbour_robot: True
convexify: True
parser: structured
symbolic: False
use_region_bit_encoding: True
multi_robot_mode: negotiation
cooperative_gr1: True
fastslow: False
only_realizability: False
recovery: False
include_heading: False
winning_livenesses: False
synthesizer: slugs
decompose: True
interactive: False

CurrentConfigName:
aliceWithRRT

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
../../city.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
bob_postOffice, 1
bob_park, 1
bob_policeStation2, 1
bob_bridge, 1
bob_square, 1
bob_groceryStore, 1
bob_tunnel, 1
bob_policeStation1, 1


======== SPECIFICATION ========

OtherRobot: # The other robot in the same workspace
bob

RegionMapping: # Mapping between region names and their decomposed counterparts
bridge = p9
square = p2
policeStation1 = p6
policeStation2 = p5
tunnel = p1
park = p7
postOffice = p4
others = 
groceryStore = p8

Spec: # Specification in structured English
Robot starts in policeStation2
Environment starts with bob_postOffice

# env goals #
#infinitely often bob_policeStation2
#infinitely often bob_groceryStore
#infinitely often bob_policeStation1
#infinitely often bob_postOffice

# env assumptions #
if you were in policeStation1 then do not bob_park
if you were in postOffice then do not bob_park
if you were in park then do not (bob_tunnel or bob_bridge or bob_policeStation1)
if you were in tunnel then do not (bob_park or bob_square)
if you were in bridge then do not (bob_park or bob_square)
if you were in square then do not (bob_tunnel or bob_bridge or bob_policeStation2)
if you were in groceryStore then do not bob_square
if you were in policeStation2 then do not bob_square

# system goals #
visit policeStation1
visit policeStation2

