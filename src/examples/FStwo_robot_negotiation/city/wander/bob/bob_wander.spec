# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)

CompileOptions:
synthesizer: slugs
fastslow: True
convexify: True
recovery: False
parser: structured
symbolic: False
decompose: True
use_region_bit_encoding: True

CurrentConfigName:
bob

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
../../city.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
alice_policeStation2, 1
alice_groceryStore, 1
alice_square, 1
alice_bridge, 1
alice_tunnel, 1
alice_postOffice, 1
alice_park, 1
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
Robot starts in groceryStore
Env starts with alice_bridge

# Environment Assumptions
if you have finished groceryStore then do not (alice_groceryStore or alice_square)
if you have finished square then do not (alice_square or alice_tunnel)
if you have finished tunnel then do not (alice_tunnel or alice_park)
if you have finished park then do not (alice_park or alice_postOffice)
if you have finished postOffice then do not alice_postOffice

######### system guarantees ##########
# not allowing both robots to be at the same place
if you are sensing alice_square then do not square
if you are sensing alice_park then do not park
if you are sensing alice_postOffice then do not postOffice
if you are sensing alice_policeStation2 then do not policeStation2
if you are sensing alice_bridge then do not bridge
if you are sensing alice_groceryStore then do not groceryStore
if you are sensing alice_tunnel then do not tunnel
if you are sensing alice_policeStation1 then do not policeStation1

visit postOffice

