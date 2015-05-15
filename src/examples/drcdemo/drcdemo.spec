# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
pickup, 1
drop, 1
standup, 0
sitdown, 0
headNod, 1
greet, 1
blinkEyes, 1
sayTicket, 1
inspect, 1
sayNo, 1

CompileOptions:
convexify: True
parser: structured
symbolic: False
use_region_bit_encoding: True
synthesizer: slugs
fastslow: False
decompose: True

CurrentConfigName:
naoActions

Customs: # List of custom propositions
gotTicket

RegionFile: # Relative path of region description file
drcdemo.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
person, 1
touchhead, 0
VIPticket, 1
NormalTicket, 0
StudentTicket, 0
ExpiredTicket, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
r1 = p2
r2 = p1
others = 

Spec: # Specification in structured English
# get ticket
do sayTicket and pickup if and only if you are sensing person and you are not activating gotTicket

# check ticket
do inspect if and only if you are activating gotTicket
if you are activating inspect and you are sensing VIPticket then do headNod and greet
if you are activating inspect and you are sensing ExpiredTicket then do blinkEyes and sayNo

# return ticket
do drop if and only if you were activating inspect and you are activating gotTicket
gotTicket is set on pickup and reset on drop

# assumptions
always not (VIPticket and ExpiredTicket)

# action constraints
always not (pickup and drop)

