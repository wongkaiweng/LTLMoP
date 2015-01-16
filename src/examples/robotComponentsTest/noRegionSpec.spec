# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
drop_um, 1
pickup_dm, 1

CompileOptions:
convexify: True
parser: structured
symbolic: False
use_region_bit_encoding: True
synthesizer: jtlv
fastslow: False
decompose: True

CurrentConfigName:
Untitled configuration

Customs: # List of custom propositions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
ball_d, 1
person_u, 1


======== SPECIFICATION ========

Spec: # Specification in structured English
if you are sensing ball_d then do pickup_dm
if you are sensing person_u then do drop_um

