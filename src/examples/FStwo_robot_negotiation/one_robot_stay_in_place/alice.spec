# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
pickup, 0
wave, 0
pushButton, 1

CompileOptions:
neighbour_robot: True
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

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
one_line_two_regions.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
doorOpened, 1


======== SPECIFICATION ========

OtherRobot: # The other robot in the same workspace

RegionMapping: # Mapping between region names and their decomposed counterparts
r1 = p2
r2 = p1
others = 

Spec: # Specification in structured English
Robot starts in r1
#Env starts with not pickup_ac and wave_ac

#always wave_ac
#do pickup if and only if you are sensing wave_ac
#visit pickup
#visit wave

# push button together to trigger sth
# button trigger some sensor event
# then door opens and the robots can go?

if you were sensing pushButton_ac and bob_pushButton_ac then do doorOpened
if you are sensing doorOpened then visit r2

