# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
pickup, 0
drop, 0
standup, 0
sitdown, 0
headNod, 0
greet, 0
sayTicket, 0
inspect, 0
sayNo, 0
headShake, 0
tweet, 0
sayTweet, 0

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

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
person, 0
touchhead, 0
VIPticket, 0
ExpiredTicket, 0


======== SPECIFICATION ========

Spec: # Specification in structured English
# action constraints
#always not (pickup and drop)
#alway not (standup and sitdown)
#always not (sayTicket and sayNo)

#always not (sayTicket and sayTweet)
#always not (sayNo and sayTweet)

#always not (headShake and headNod)

