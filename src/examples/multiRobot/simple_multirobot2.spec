# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
camera, 0

CompileOptions:
convexify: False
parser: ltl
symbolic: False
use_region_bit_encoding: False
synthesizer: slugs
fastslow: True
decompose: True

CurrentConfigName:
threeRobots

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
simpleLTL.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
cameraOn, 0
person, 0


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
others = 
r1 = p3
r2 = p2
r3 = p1

Spec: # Specification in structured English
# Original Spec (ENV part)
e.rob1_r1_rc

[]<>(s.rob1_r1  -> next(e.rob1_r1_rc))
[]<>(s.rob1_r2  -> next(e.rob1_r2_rc))
[]<>(s.rob1_r3  -> next(e.rob1_r3_rc))

#phi_e_new
[](next(e.rob1_r1_rc) <-> !next(e.rob1_r2_rc) & !next(e.rob1_r3_rc))
[](next(e.rob1_r3_rc) <-> !next(e.rob1_r1_rc) & !next(e.rob1_r2_rc))
[](next(e.rob1_r2_rc) <-> !next(e.rob1_r3_rc) & !next(e.rob1_r1_rc))

[](next(e.rob1_r1_rc) | next(e.rob1_r2_rc) | next(e.rob1_r3_rc))

[]((e.rob1_r1_rc & s.rob1_r1)-> next(e.rob1_r1_rc))
[]((e.rob1_r2_rc & s.rob1_r2)-> next(e.rob1_r2_rc))
[]((e.rob1_r3_rc & s.rob1_r3)-> next(e.rob1_r3_rc))
--
# Original Spec (SYS part)
s.rob1_r2

[]<> (e.rob1_r1_rc)
[]<> (e.rob1_r2_rc)
[]<> (e.rob1_r3_rc)

