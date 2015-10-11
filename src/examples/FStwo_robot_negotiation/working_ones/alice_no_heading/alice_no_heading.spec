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
fastslow: True
only_realizability: False
recovery: False
include_heading: False
winning_livenesses: False
synthesizer: slugs
decompose: True
interactive: False

CurrentConfigName:
aliceWithBobSensorR5

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
../two_robot_negotiation.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
bob_r1, 1
bob_r2, 1
bob_r3, 1
bob_r4, 1
bob_r5, 1
bob_r6, 1


======== SPECIFICATION ========

OtherRobot: # The other robot in the same workspace
bob

RegionMapping: # Mapping between region names and their decomposed counterparts
r4 = p3
r5 = p2
r6 = p1
r1 = p6
r2 = p5
r3 = p4
others = 

Spec: # Specification in structured English
####### initial conditions ##########
Robot starts in r5
Environment starts with bob_r1

###### environment assumptions ######
#if you have finished r5 then do not (bob_r5 or bob_r4)
#if you have finished r4 then do not (bob_r4 or bob_r2)

if you have finished r5 then do not (bob_r5 or bob_r3)
if you have finished r3 then do not (bob_r2 or bob_r3)

if you have finished r2 then do not (bob_r1 or bob_r2)
if you have finished r1 then do not ( bob_r1)
#infinitely often not bob_r1
#infinitely often not bob_r2
#infinitely often not bob_r3
#infinitely often not bob_r4
#infinitely often not bob_r5

######### system guarantees ##########
# not allowing both robots in the same region
if you are sensing bob_r1 then do not r1
if you are sensing bob_r2 then do not r2
if you are sensing bob_r3 then do not r3
if you are sensing bob_r4 then do not r4
if you are sensing bob_r5 then do not r5
if you are sensing bob_r6 then do not r6

######## system goals ###########
visit r1

#$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$#
#$$ Tried Enviornment assumptions $$#
#$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$#
# guarantee the path is clear to r1
# Trial 1
#if you were in r3 then do not bob_r2
#if you were in r2 then do not bob_r1

# Trial 2
#if you were activating r2 then do not bob_r2
#if you were activating r1 then do not bob_r1

# Trial 3
#if you are activating r2 then do not bob_r2
#if you are activating r1 then do not bob_r1

# Trial 4
# involves manually changing trial 3 to present tense but doesnt work

# Trial 5
#if you had finished r3 then do not bob_r2
#if you had finished r2 then do not bob_r1

# Trial 6
#if you had finished r2 then do not bob_r2
#if you had finished r1 then do not bob_r1

#if you were activating r3 then do not bob_r3
#if you were activating r2 then do not bob_r2
#if you were activating r1 then do not bob_r1
#if you were activating r4 then do not bob_r4
#if you were activating r5 then do not bob_r5

#$$ WORKING ONES $$#
#if you have finished r1 then do not bob_r1
#if you have finished r2 then do not bob_r2
#if you have finished r3 then do not bob_r3
#if you have finished r4 then do not bob_r4
#if you have finished r5 then do not bob_r5

#if you had finished r3 then do not (bob_r2 or bob_r3 or bob_r5)
#if you had finished r2 then do not (bob_r1 or bob_r2 or bob_r3 or bob_r4)
#if you had finished r5 then do not (bob_r4 or bob_r5 or bob_r3)
#if you had finished r4 then do not (bob_r2 or bob_r4 or bob_r5)
#if you had finished r1 then do not (bob_r2 or bob_r1)

