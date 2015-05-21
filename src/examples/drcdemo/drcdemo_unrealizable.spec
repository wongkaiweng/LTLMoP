# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
pickup, 1
drop, 1
standup, 1
sitdown, 1
headNod, 0
greet, 1
sayTicket, 0
inspect, 0
sayNo, 0
headShake, 0

CompileOptions:
convexify: True
parser: structured
symbolic: False
use_region_bit_encoding: True
synthesizer: jtlv
fastslow: False
decompose: True

CurrentConfigName:
naoActions

Customs: # List of custom propositions
gotTicket

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
person, 1
touchhead, 1
VIPticket, 1
ExpiredTicket, 0


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
r1 = p2
r2 = p1
others = 

Spec: # Specification in structured English
If you are sensing person then do standup
if you are sensing touchhead then do sitdown

If you are sensing VIPticket then do greet and pickup
If you are not sensing VIPticket then do drop

# action constraints
always not (standup and sitdown)
always not (pickup and drop)

