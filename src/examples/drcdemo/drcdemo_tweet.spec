# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
pickup, 1
drop, 0
standup, 0
sitdown, 0
headNod, 0
greet, 0
sayTicket, 0
inspect, 0
sayNo, 0
sayInvalid, 0
sayHello, 0
sayPlay, 0
sayCheckResearch, 0
fistBump, 0
highFive, 0 
headShake, 0
tweet, 1
sayTweet, 1

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
detectVIP

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
person, 1
touchhead, 1
VIPticket, 0
ExpiredTicket, 0


======== SPECIFICATION ========

Spec: # Specification in structured English
# detectVIP
detectVIP is set on person and reset on tweet
if you are sensing person then do sayTweet
do tweet if and only if you are activating detectVIP and you are sensing touchhead

