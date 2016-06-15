# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
pickup, 0
drop, 0
standup, 1
sitdown, 1
headNod, 0
greet, 0
sayTicket, 0
inspect, 0
sayNo, 0
sayInvalid, 0
sayHello, 1
sayPlay, 0
sayCheckResearch, 0
headShake, 0
highFive, 1
fistBump, 1
tweet, 0
sayTweet, 0

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
ExpiredTicket, 1


======== SPECIFICATION ========

Spec: # Specification in structured English
# action constraints
#always not (pickup and drop)
#alway not (standup and sitdown)
#always not (sayTicket and sayNo)

#always not (sayTicket and sayTweet)
#always not (sayNo and sayTweet)

#always not (headShake and headNod)

if you are sensing person then do sayHello

if you are sensing touchhead then do fistBump

if you are sensing VIPticket then do sitdown

if you are sensing ExpiredTicket then do standup

