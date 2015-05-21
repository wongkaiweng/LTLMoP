# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
pickup, 1
drop, 1
standup, 0
sitdown, 0
headNod, 0
greet, 0
sayTicket, 0
inspect, 0
sayNo, 0
headShake, 0
tweet, 1
sayTweet, 1

CompileOptions:
convexify: True
parser: structured
symbolic: False
use_region_bit_encoding: True
synthesizer: slugs
fastslow: False
decompose: True

Customs: # List of custom propositions
detectVIP

CurrentConfigName:
naoActions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
person, 0
touchhead, 1
VIPticket, 1
ExpiredTicket, 0


======== SPECIFICATION ========

Spec: # Specification in structured English
# detectVIP 
detectVIP is set on VIPticket and reset on tweet
if you are sensing VIPticket then do sayTweet
do tweet if and only if you are activating detectVIP and you are sensing touchhead



