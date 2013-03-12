# This is a specification definition file for the LTLMoP toolkit.
# Format details are described at the beginning of each section below.


======== SETTINGS ========

Actions: # List of action propositions and their state (enabled = 1, disabled = 0)
pick_up, 1
deliver, 1

CompileOptions:
convexify: True
fastslow: False

CurrentConfigName:
basicsim

Customs: # List of custom propositions
carrying_food
c1_needs_food
c2_needs_food

RegionFile: # Relative path of region description file
restaurant.regions

Sensors: # List of sensor propositions and their state (enabled = 1, disabled = 0)
food_ready, 1
c1_order, 1
c2_order, 1


======== SPECIFICATION ========

RegionMapping: # Mapping between region names and their decomposed counterparts
c1 = p5
kitchen = p3
others = p6, p7, p8, p9, p10, p11, p12, p13, p14, p15, p16
c2 = p4

Spec: # Specification in structured English
robot starts with false

# Assume we have a cooperative kitchen
infinitely often food_ready

# How to get food
# Note: you would expect "food_ready" to be in the Set condition here
# too, but the parser puts it in the next() tense which is wrong
carrying_food is set on kitchen and pick_up and reset on deliver
if you are not sensing food_ready then do not pick_up

# We can only carry one thing at a time
if you are activating carrying_food then do not pick_up

# We need food to deliver
if you are not activating carrying_food then do not deliver

# NOTE: we do not explicitly prevent the robot from spuriously delivering,
# but it would be "inefficient" to do so anyways.

# Satisfy all orders
c1_needs_food is set on c1_order and reset on c1 and deliver
c2_needs_food is set on c2_order and reset on c2 and deliver
infinitely often not c1_needs_food
infinitely often not c2_needs_food

if you are sensing c1_order then stay there

# Note: If we use Bingxin's memory extensions, this last set could just be:
# after each time c1_order, visit c1 and deliver
# after each time c2_order, visit c2 and deliver

# Instantaneous reaction stuff
if you were activating start of c1_needs_food then stay there
if you were activating start of c2_needs_food then stay there

