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

CurrentConfigName:
aliceWithBobSensor

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
../two_robot_negotiation.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
bob_r1, 1
bob_r2, 1
bob_r3, 1
bob_r4, 1
bob_r5, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
r4 = p2
r5 = p1
r1 = p5
r2 = p4
r3 = p3
others = 

Spec: # Specification in structured English
####### initial conditions ##########
Robot starts in r3
Environment starts with bob_r1

# ----- envChar ----------#
#infinitely often not bob_r1

# ---- Fairness -------- #
infinitely often bob_r5

###### environment assumptions ######
# johnny5 can only be at one region at a time
always (bob_r1 and not bob_r2 and not bob_r3 and not bob_r4 and not bob_r5) or (not bob_r1 and bob_r2 and not bob_r3 and not bob_r4 and not bob_r5) or (not bob_r1 and not bob_r2 and bob_r3 and not bob_r4 and not bob_r5) or (not bob_r1 and not bob_r2 and not bob_r3 and bob_r4 and not bob_r5) or (not bob_r1 and not bob_r2 and not bob_r3 and not bob_r4 and bob_r5)

# transition assumptions
if you were sensing bob_r1 then do (bob_r1 or bob_r2)
if you were sensing bob_r2 then do (bob_r1 or bob_r2 or bob_r3 or bob_r4)
if you were sensing bob_r3 then do (bob_r2 or bob_r3 or bob_r5)
if you were sensing bob_r4 then do (bob_r2 or bob_r4 or bob_r5)
if you were sensing bob_r5 then do (bob_r3 or bob_r4 or bob_r5)

# guarantee the path is clear to r1
if you were in r3 then do not bob_r2
if you were in r2 then do not bob_r1

######### system guarantees ##########
# not allowing both robots to be at the same place
if you are sensing bob_r1 then do not r1
if you are sensing bob_r2 then do not r2
if you are sensing bob_r3 then do not r3
if you are sensing bob_r4 then do not r4
if you are sensing bob_r5 then do not r5

# -------------- from bob ----------------------#
if you were sensing bob_r1 then do not r2
if you were sensing bob_r2 then do not r4
if you were sensing bob_r4 then do not r5

# -------------- from bob ----------------------#
#infinitely often not r2

######## system goals ###########
visit r1

