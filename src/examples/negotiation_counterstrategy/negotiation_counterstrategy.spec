# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
camera, 0

CompileOptions:
convexify: True
parser: structured
symbolic: False
use_region_bit_encoding: True
synthesizer: slugs
fastslow: False
decompose: True

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
negotiation_counterstrategy.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
ball, 1
cat, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
r1 = p2
r2 = p1
others = 

Spec: # Specification in structured English
robot starts in r1

# ---------------------------
visit r2
if you are sensing ball then do not r2
# counterstrategy for only ball
#always not ball

if you are sensing cat then do not r1
visit r1
# counterStrategy for ball and cat
#always not ball and not cat

# -------
#if you are sensing ball then do camera
#if you are sensing ball then do not camera
#if you were not in r2 then do not ball

#if you are sensing (camera or r2 or not ball) then do not ball

#if you were not sensing (r2 and ball) then do not ball

