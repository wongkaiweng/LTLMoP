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
bobWithRRT

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
../../city.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
alice_postOffice, 1
alice_park, 1
alice_policeStation2, 1
alice_bridge, 1
alice_square, 1
alice_groceryStore, 1
alice_tunnel, 1
alice_policeStation1, 1


======== SPECIFICATION ========

OtherRobot: # The other robot in the same workspace
alice

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
# Init conditions #
Robot starts in postOffice
Environment starts with alice_policeStation2

# env assumptions #
If you were in postOffice then do not alice_park
if you were in policeStation1 then do not alice_park
If you were in park then do not (alice_tunnel or alice_bridge or alice_postOffice)
if you were in tunnel then do not (alice_square or alice_park)
if you were in bridge then do not (alice_square or alice_park)
if you were in square then do not (alice_groceryStore or alice_tunnel or alice_bridge)
if you were in groceryStore then do not alice_square
if you were in policeStation2 then do not alice_square

# system goals #
visit groceryStore
visit postOffice

