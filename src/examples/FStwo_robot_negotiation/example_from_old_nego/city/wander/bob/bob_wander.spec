# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)

CompileOptions:
convexify: True
parser: structured
symbolic: False
use_region_bit_encoding: True
synthesizer: slugs
fastslow: True
decompose: True

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
#if you had finished groceryStore then do not alice_square
#if you had finished square then do not alice_tunnel
#if you had finished tunnel then do not alice_park
#if you had finished park then do not alice_postOffice

if you had finished groceryStore or you were activating groceryStore then do not alice_square
if you had finished square or you were activating square then do not alice_tunnel
if you had finished tunnel or you were activating tunnel then do not alice_park
if you had finished park or you were activating park then do not alice_postOffice


visit postOffice

