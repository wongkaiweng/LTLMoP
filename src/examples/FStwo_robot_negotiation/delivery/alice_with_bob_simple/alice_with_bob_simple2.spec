# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
pickup, 1
deliver, 1

CompileOptions:
neighbour_robot: True
convexify: True
parser: structured
symbolic: False
use_region_bit_encoding: True
multi_robot_mode: negotiation
cooperative_gr1: True
fastslow: True
only_realizability: False
recovery: True
include_heading: False
winning_livenesses: False
synthesizer: slugs
decompose: True
interactive: True

CurrentConfigName:
basicSim

Customs: # List of custom propositions

RegionFile: # Relative path of region description file
../threeCorridorsShort.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
itemRequest, 1
bob_hallwayTop, 1
bob_office, 1
bob_reception, 1
bob_cafe, 1
bob_emergencyExit, 1
bob_hallwayBottom, 1
bob_hallwayCentral, 1
bob_library, 1
bob_atrium, 1
bob_storageBottom, 1
bob_storageTop, 1
itemReceived, 1


======== SPECIFICATION ========

GlobalSensors: # Sensors accessible by all robots

OtherRobot: # The other robot in the same workspace
bob

RegionMapping: # Mapping between region names and their decomposed counterparts
emergencyExit = p11
hallwayBottom = p8
office = p4
storageTop = p1
storageBottom = p2
hallwayTop = p6
library = p5
reception = p3
hallwayCentral = p7
atrium = p13
cafe = p12
others = 

Spec: # Specification in structured English
Robot starts in library
Environment starts with bob_emergencyExit
#always (not blockTop and not blockMiddle and blockBottom) or  (not blockTop and blockMiddle and not blockBottom) or (not blockTop and not blockMiddle and not blockBottom)
# or (blockTop and not blockMiddle and not blockBottom)

# don't go to hallways when they are blocked
#if you are sensing blockTop then do not hallwayTop
#if you are sensing blockMiddle then do not hallwayCentral
#if you are sensing blockBottm then do not hallwayBottom

# pick up and deliver things
#if you are sensing itemRequest and you have finished library then do pickup
do pickup if and only if you are sensing itemRequest and you have finished library
if you are sensing itemReceived then visit office
do deliver if and only if you are sensing itemReceived and you have finished office
if you are activating deliver then stay there
if you are not sensing itemReceived then visit library
#visit hallwayCentral


if you have finished library then do not (bob_library or bob_atrium or bob_hallwayCentral or bob_storageBottom or bob_storageTop or bob_hallwayTop or bob_hallwayBottom)
if you have finished atrium then do not (bob_library or bob_atrium or bob_hallwayCentral or bob_reception )
if you have finished hallwayCentral then do not (bob_library or bob_atrium or bob_hallwayCentral or bob_reception or bob_office or bob_storageTop or bob_storageBottom or bob_cafe or bob_emergencyExit)
if you have finished reception then do not (bob_hallwayCentral or bob_reception or bob_office or bob_atrium)
if you have finished office then do not (bob_office or bob_reception or bob_hallwayCentral or bob_emergencyExit or bob_cafe or bob_hallwayTop or bob_hallwayBottom)

####### assumptions added for this nego (extra) - two steps
if you have finished atrium then do not (bob_storageBottom or bob_storageTop or bob_hallwayBottom or bob_hallwayTop)
if you have finished storageTop then do not (bob_atrium or bob_storageTop or bob_hallwayTop or bob_storageBottom or bob_emergencyExit)
if you have finished storageBottom then do not (bob_atrium or bob_storageBottom or bob_hallwayBottom or bob_storageTop or bob_cafe)
if you have finished hallwayBottom then do not (bob_cafe or bob_storageBottom or bob_hallwayBottom or bob_reception or bob_atrium)
if you have finished hallwayTop then do not (bob_emergencyExit or bob_storageTop or bob_hallwayTop or bob_reception or bob_atrium)
if you have finished cafe then do not (bob_reception or bob_cafe or bob_hallwayBottom or bob_storageBottom or bob_emergencyExit)
if you have finished emergencyExit then do not (bob_emergencyExit or bob_reception or bob_hallwayTop or bob_storageTop or bob_cafe)
if you have finished reception then do not (bob_emergencyExit or bob_cafe or bob_hallwayTop or bob_hallwayBottom)


# after nego sysTrans (new)
#if you are sensing bob_atrium then do not (finished atrium or finished storageTop)
#if you are sensing bob_storageTop then do not (finished storageTop or finished hallwayTop)
#if you are sensing bob_hallwayTop then do not (finished hallwayTop or finished emergencyExit)
#if you are sensing bob_emergencyExit then do not (finished reception or finished emergencyExit)
#if you are sensing bob_reception then do not (finished reception or finished cafe)
#if you are sensing bob_cafe then do not (finished hallwayBottom or finished cafe)
#if you are sensing bob_hallwayBottom then do not (finished hallwayBottom or finished cafe)
#if you are sensing bob_storageBottom then do not (finished storageBottom or finished atrium)

#infinitely often bob_hallwayTop
#infinitely often bob_hallwayBottom



#### sysTrans added for --- restricting bob to be only on top --- ####
#if you are sensing bob_atrium then do not (atrium or storageTop or storageBottom)
#if you are sensing bob_reception then do not (reception or emergencyExit or cafe)
#if you are sensing bob_storageBottom then do not (storageBottom or hallwayBottom or atrium)
#if you are sensing bob_hallwayBottom then do not (hallwayBottom or storageBottom or cafe)
#if you are sensing bob_cafe then do not (cafe or hallwayBottom or reception)
#if you are sensing bob_emergencyExit then do not (emergencyExit or hallwayTop or reception)
#if you are sensing bob_hallwayTop then do not (storageTop or hallwayTop or emergencyExit)
#if you are sensing bob_storageTop then do not (storageTop or hallwayTop or atrium)

###------------------------------------------------------ ###
###-- restricting bob to be only on top--------- ###
###------------------------------------------------------ ###
#always bob_emergencyExit

#if you have finished library then do not (bob_library or bob_atrium or bob_hallwayCentral)
#if you have finished atrium then do not (bob_library or bob_atrium or bob_hallwayCentral or bob_reception )
#if you have finished hallwayCentral then do not (bob_library or bob_atrium or bob_hallwayCentral or bob_reception or bob_office)
#if you have finished reception then do not (bob_hallwayCentral or bob_reception or bob_office or bob_atrium)
#if you have finished office then do not (bob_office or bob_reception or bob_hallwayCentral)

#if you have finished library then do (bob_emergencyExit or bob_reception)
#if you have finished atrium then do (bob_emergencyExit)
#if you have finished hallwayCentral then do (bob_hallwayTop)
#if you have finished reception then do (bob_storageTop)
#if you have finished office then do (bob_storageTop or bob_atrium)

#if you are sensing bob_storageBottom then do not (storageBottom or hallwayBottom or atrium)
#if you are sensing bob_hallwayBottom then do not (hallwayBottom or storageBottom or cafe or atrium or reception)
#if you are sensing bob_cafe then do not (cafe or hallwayBottom or reception)
#if you are sensing bob_emergencyExit then do not (emergencyExit or hallwayTop or reception)
#if you are sensing bob_hallwayTop then do not (storageTop or hallwayTop or emergencyExit or atrium or reception)
#if you are sensing bob_storageTop then do not (storageTop or hallwayTop or atrium)




####### assumptions added for this nego (extra) - one step
#if you have finished atrium then do not (bob_storageBottom or bob_storageTop)
#if you have finished storageTop then do not (bob_atrium or bob_storageTop or bob_hallwayTop)
#if you have finished storageBottom then do not (bob_atrium or bob_storageBottom or bob_hallwayBottom)
#if you have finished hallwayBottom then do not (bob_cafe or bob_storageBottom or bob_hallwayBottom)
#if you have finished hallwayTop then do not (bob_emergencyExit or bob_storageTop or bob_hallwayTop)
#if you have finished cafe then do not (bob_reception or bob_cafe or bob_hallwayBottom)
#if you have finished emergencyExit then do not (bob_emergencyExit or bob_reception or bob_hallwayTop)
#if you have finished reception then do not (bob_emergencyExit or bob_cafe)


# env maintain distance
#if you are sensing bob_atrium then do (finished hallwayCentral or finished library or finished office or finished emergencyExit or finished reception or finished cafe or finished hallwayTop or finished hallwayBottom )
#if you are sensing bob_storageTop then do (finished hallwayCentral or finished library or finished office or finished reception or finished hallwayBottom or finished cafe or finished storageBottom or finished emergencyExit)
#if you are sensing bob_hallwayTop then do (finished hallwayCentral or finished library or finished office or finished cafe or finished hallwayBottom or finished storageBottom or finished atrium or finished reception)
#if you are sensing bob_emergencyExit then do (finished hallwayCentral or finished library or finished office or finished hallwayBottom or finished storageBottom or finished atrium or finished cafe or finished storageTop)
#if you are sensing bob_reception then do (finished hallwayCentral or finished library or finished office  or finished storageTop or finished storageBottom or finished atrium or finished hallwayTop or finished hallwayBottom)
#if you are sensing bob_cafe then do (finished hallwayCentral or finished library or finished office  or finished storageTop or finished hallwayTop or finished atrium or finished emergencyExit or finished storageBottom)
#if you are sensing bob_hallwayBottom then do (finished hallwayCentral or finished library or finished office or finished storageTop or finished hallwayTop or finished emergencyExit or finished storageBottom or finished atrium or finished reception)
#if you are sensing bob_storageBottom then do (finished hallwayCentral or finished library or finished office or finished reception or finished hallwayTop or finished emergencyExit or finished storageTop or finished cafe)


# sys maintain distance
#if you are sensing bob_atrium then do (hallwayCentral or library or office or emergencyExit or reception or cafe)
#if you are sensing bob_storageTop then do (hallwayCentral or library or office or reception or hallwayBottom or cafe)
#if you are sensing bob_hallwayTop then do (hallwayCentral or library or office or cafe or hallwayBottom or storageBottom)
#if you are sensing bob_emergencyExit then do (hallwayCentral or library or office or hallwayBottom or storageBottom or atrium)
####if you are sensing bob_reception then do (hallwayCentral or library or office or storageTop or storageBottom or atrium)
######if you are sensing bob_cafe then do (hallwayCentral or library or office or storageTop or hallwayTop or atrium)
#if you are sensing bob_hallwayBottom then do (hallwayCentral or library or office or storageTop or hallwayTop or emergencyExit)#
#if you are sensing bob_storageBottom then do (hallwayCentral or library or office or reception or hallwayTop or emergencyExit)

