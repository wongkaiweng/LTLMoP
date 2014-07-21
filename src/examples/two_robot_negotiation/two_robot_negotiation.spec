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
fastslow: False
decompose: True

CurrentConfigName:
BasicSim

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
two_robot_negotiation.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
johnny_r1, 1
johnny_r2, 1
johnny_r3, 1
johnny_r4, 1
johnny_r5, 1


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
Robot starts in r1
Environment starts with johnny_r3

###### environment assumptions ######
# johnny5 can only be at one region at a time
always (johnny_r1 and not johnny_r2 and not johnny_r3 and not johnny_r4 and not  johnny_r5) or (not johnny_r1 and johnny_r2 and not johnny_r3 and not johnny_r4 and not  johnny_r5) or (not johnny_r1 and not johnny_r2 and johnny_r3 and not johnny_r4 and not  johnny_r5) or (not johnny_r1 and not johnny_r2 and not johnny_r3 and  johnny_r4 and not  johnny_r5) or (not johnny_r1 and not johnny_r2 and not johnny_r3 and not johnny_r4 and   johnny_r5)

# guarantee the path is clear to r5
if you were in r1 then do not johnny_r2
if you were in r2 then do not johnny_r4
if you were in r4 then do not johnny_r5

######### system guarantees ##########
# not allowing both robots to be at the same place
if you are sensing johnny_r1 then do not r1
if you are sensing johnny_r2 then do not r2
if you are sensing johnny_r3 then do not r3
if you are sensing johnny_r4 then do not r4
if you are sensing johnny_r5 then do not r5

######## system goals ###########
visit r5

