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
sayTicket, 1
inspect, 1
sayNo, 1
headShake, 1
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
person, 1
touchhead, 0
VIPticket, 1
ExpiredTicket, 1


======== SPECIFICATION ========

Spec: # Specification in structured English
# get ticket
do sayTicket and pickup if and only if you are sensing person and you are not activating gotTicket

# check ticket
do inspect if and only if you are activating gotTicket
do headNod and greet if and only if you are activating gotTicket and you are sensing VIPticket
do headShake and sayNo if and only if you are activating gotTicket and you are sensing ExpiredTicket

# return ticket
do drop if and only if you were activating (headNod or headShake) and you are activating gotTicket
gotTicket is set on pickup and reset on drop

# assumptions
always not (VIPticket and ExpiredTicket)

# action constraints
always not (pickup and drop)

