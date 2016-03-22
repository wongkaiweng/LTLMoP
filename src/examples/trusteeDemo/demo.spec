# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
standup, 1
sitdown, 1
sayGoodMorning, 1

CompileOptions:
convexify: True
parser: structured
symbolic: False
use_region_bit_encoding: True
synthesizer: jtlv
fastslow: False
decompose: True

CurrentConfigName:
nao

Customs: # List of custom propositions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
Hadas, 1
headTapped, 1


======== SPECIFICATION ========

Spec: # Specification in structured English
if you are sensing Hadas then do standup and sayGoodMorning
if you are sensing headTapped and not Hadas then do sitdown

always not (standup and sitdown)

