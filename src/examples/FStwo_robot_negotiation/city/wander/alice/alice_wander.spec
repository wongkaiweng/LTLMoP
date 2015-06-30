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
alice

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
../../city.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
bob_policeStation2, 1
bob_groceryStore, 1
bob_bridge, 1
bob_tunnel, 1
bob_postOffice, 1
bob_park, 1
bob_policeStation1, 1
bob_square, 1


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
Robot starts in bridge
Env starts with bob_groceryStore

###### environment assumptions ######
if you have finished bridge then do not (bob_bridge or bob_square)
if you have finished square then do not (bob_square or bob_groceryStore)
if you have finished groceryStore then do not (bob_groceryStore)

######### system guarantees ##########
# not allowing both robots to be at the same place
if you are sensing bob_square then do not square
if you are sensing bob_park then do not park
if you are sensing bob_postOffice then do not postOffice
if you are sensing bob_policeStation2 then do not policeStation2
if you are sensing bob_bridge then do not bridge
if you are sensing bob_groceryStore then do not groceryStore
if you are sensing bob_tunnel then do not tunnel
if you are sensing bob_policeStation1 then do not policeStation1

visit groceryStore

# and not bob_square and not bob_park and not bob_postOffice and not bob_policeStation2 and not bob_bridge and not bob_tunnel and not bob_policeStation1

# Environment Assumptions
#if you had finished bridge then do not bob_square
#if you had finished square then do not bob_groceryStore

#if you were activating square or you had finished square then do not bob_square
#if you were activating groceryStore or you had finished groceryStore then do not bob_groceryStore

