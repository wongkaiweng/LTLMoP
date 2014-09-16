# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)

CompileOptions:
convexify: True
parser: structured
symbolic: False
use_region_bit_encoding: True
synthesizer: jtlv
fastslow: False
decompose: True

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
../city.regions

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
bridge = p10
square = p2
policeStation1 = p6
policeStation2 = p5
tunnel = p1
park = p8
postOffice = p4
others = 
groceryStore = p9

Spec: # Specification in structured English
# Init conditions #
Robot starts in postOffice
Environment starts with alice_policeStation1

#always alice_policeStation2

# goals
infinitely often alice_policeStation1
infinitely often alice_policeStation2

# env assumptions #
#If you were in postOffice then do not alice_park
#If you were in park then do not (alice_tunnel or alice_policeStation1 or alice_bridge)
#if you were in tunnel then do not alice_square
#if you were in bridge then do not alice_square
#if you were in square then do not (alice_groceryStore or alice_policeStation2 or alice_tunnel or alice_bridge)

# system goals #
visit policeStation1
visit policeStation2
visit groceryStore
visit postOffice

