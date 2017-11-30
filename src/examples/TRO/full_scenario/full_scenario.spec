# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
sayAlarm, 1
saySpill, 1
pickup, 1
drop, 1

CompileOptions:
neighbour_robot: False
convexify: False
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
maecy

Customs: # List of custom propositions
informManager
gotMetal
gotGlass
gotPaper

RegionFile: # Relative path of region description file
full_scenario.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
alarm, 1
headTapped, 1
spill, 1
metal, 1
glass, 1
paper, 1


======== SPECIFICATION ========

GlobalSensors: # Sensors accessible by all robots

OtherRobot: # The other robot in the same workspace

RegionMapping: # Mapping between region names and their decomposed counterparts
entrance = p6
metal_deposit = p4
glass_deposit = p5
office = p3
paper_deposit = p2
others = p1

Spec: # Specification in structured English
#Robot starts in office

########################
## inform Manager on alarm ###
########################
informManager is set on alarm and not finished office and not spill and reset on headTapped and finished office
if you have finished office and you are activating informManager then do sayAlarm
if you have finished office and you are activating informManager then stay there

# ** assumptions (turn off alarm before headTapped)
##if you were activating informManager and you were sensing headTapped then do not alarm
always not (alarm and headTapped)
#----------------------------------------#

############
#### Spill ####
############
if you are sensing spill then do saySpill
if you are sensing spill then stay there

# ** assumptions (manager would have cleaned up spill)
if you were activating informManager then do not spill

# ++ liveness (if assumptions failed)
##infinitely often not informManager or (not spill and finished office and office)
##infinitely often not spill and finished office
#infinitely often not spill and not informManager
#----------------------------------------#

##########
## Patrol ##
##########
if you are not activating (informManager or spill or gotMetal or gotGlass or gotPaper or metal or paper or glass) then visit entrance
if you are not activating (informManager or spill or gotMetal or gotGlass or gotPaper or metal or paper or glass) then visit office
if you are activating informManager then visit office
#----------------------------------------#

#always not (gotMetal or gotGlass or gotPaper)
#################
## Pickup and Drop ###
#################

##++ liveness
##infinitely often not pickup or finished pickup
##infinitely often not drop or finished drop
##infinitely often not ((drop and gotMetal)) or (finished drop and finished metal_deposit)
##infinitely often not ((drop and gotPaper)) or (finished drop and finished paper_deposit)
##infinitely often not ((drop and gotGlass)) or (finished drop and finished glass_deposit)



## ** assumptions
##if you were activating gotMetal or gotGlass then do not (glass or metal)
##if you were activating gotMetal then do not metal
##if you were activating gotPaper then do not paper
##if you were activating gotGlass then do not glass
##always not ((glass and metal) or (metal and paper) or (paper and glass))
if you have finished metal_deposit then do not (glass or paper)
if you have finished glass_deposit then do not (metal or paper)
if you have finished paper_deposit then do not (glass or metal)

# pickup object if you see them
do pickup if and only if you are sensing (metal or glass or paper) and you are not activating (gotMetal or gotGlass or gotPaper or informManager)
if you are activating pickup then stay there
#if you are sensing (metal or glass or paper) then stay there

# remember what you have picked up
gotMetal is set on metal and finished pickup and reset on finished metal_deposit and finished drop
gotGlass is set on glass and finished pickup and reset on finished glass_deposit and finished drop
gotPaper is set on paper and finished pickup and reset on finished paper_deposit and finished drop

# drop the object at the right place
do drop if and only if you were sensing (gotMetal and finished metal_deposit) or (gotGlass and finished glass_deposit) or (gotPaper and finished paper_deposit) and you were not activating informManager
if you are activating drop then stay there

if you are not activating (informManager or gotGlass or gotPaper) and you are not sensing spill and you are activating gotMetal then visit metal_deposit
if you are not activating (informManager or gotMetal or gotPaper) and you are not sensing spill and you are activating gotGlass then visit glass_deposit
if you are not activating (informManager or gotMetal or gotGlass) and you are not sensing spill and you are activating gotPaper then visit paper_deposit

