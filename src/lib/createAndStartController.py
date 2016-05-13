#!/usr/bin/env python

import subprocess
import ast, json
import sys, os
# Climb the tree to find out where we are
p = os.path.abspath(__file__)
t = ""
while t != "src":
    (p, t) = os.path.split(p)
    if p == "":
        print "I have no idea where I am; this is ridiculous"
        sys.exit(1)
sys.path.append(os.path.join(p,"src","lib"))
import copy
import time

# logger for ltlmop
import logging
ltlmop_logger = logging.getLogger('ltlmop_logger')

import project, regions, specCompiler
import hsubConfigObjects, handlerSubsystem
import lib.handlers.handlerTemplates as handlerTemplates


"""
TODO:
set $LTLMoP_dir - directory to LTLMoP  in this format: ??/??/LTLMoP/

This function takes in a .json file. Then
1. create a region file with lane-transition removed list, and .ltl to indicate direction
2. create a spec file, (LTL and SMV too)
3. create a config file
4. start execution
"""
ltlmop_logger.log(4, '== Start counting time ==')
start_time = time.time()

############################################################
# if needed, they can be changed to system arguments
scale = 500
offset = [500,500]

json_file = "/home/catherine/Desktop/fmrbenchmark/domains/dubins_traffic/dubins_traffic_utils/examples/trialsconf/mc-small-4grid-agents2.json"
with open(json_file,'r') as f:
    json_dict = ast.literal_eval(f.read())
f.closed

# TODO: this should be provided
goal_list = ["segment_4_top_lane", "segment_14_top_lane", "segment_3_pi2_right_intersect"] # given?
start_list = ["segment_2_bottom_lane"]

############################################################

LTLMoP_dir = os.environ['LTLMoP_dir']
proj = project.Project()
proj.project_root = LTLMoP_dir+"src/examples/fmr_autospec/"
proj.project_basename = "fmr_autospec"
region_basename = "road_network"

# add directory if it does not exist
if not os.path.exists(proj.project_root):
    os.makedirs(proj.project_root)

region_output_filename = proj.project_root+region_basename+".regions"
spec_filename = proj.project_root+proj.project_basename+".spec"

############################
### set compile options ####
############################
proj.compile_options = {"convexify": False, # Decompose workspace into convex regions
                        "fastslow": True, # Enable "fast-slow" synthesis algorithm
                        "symbolic": False, # Use BDDs instead of explicit-state strategies
                        "interactive": True, # Use interactive Strategy in SLUGS
                        "only_realizability": False, # only check if the spec is realizable or not
                        "decompose": False, # Create regions for free space and region overlaps (required for Locative Preposition support)
                        "use_region_bit_encoding": True, # Use a vector of "bitX" propositions to represent regions, for efficiency
                        "synthesizer": "slugs", # Name of synthesizer to use ("jtlv" or "slugs")
                        "parser": "structured", # Spec parser: SLURP ("slurp"), structured English ("structured"), or LTL ("ltl")
                        "recovery": False, # adding recovery transitions in synthesis is set to be false
                        "winning_livenesses": False, # outputs each sysGoal conjuncted with the set of winning positions
                        "cooperative_gr1": True, # synthesizing aut that phi_e is always satisfied.
                        "neighbour_robot": False, # if we will include neighbour robot LTL in the spec
                        "include_heading": False, # if we include the heading information of the other robot in the specification
                        "multi_robot_mode":"negotiation"} # Name of mode ("negotiation" or "patching")

######################################
##### create and load region file ####
######################################
# first create json_file_rnd for usage
json_file_rnd = proj.project_root+proj.project_basename+".json"
f = open(json_file_rnd,'w+')
f.write(json.dumps(json_dict["rnd"]))
f.close()

json_file_egents = proj.project_root+proj.project_basename+"_eagents.json"
f = open(json_file_egents,'w+')
f.write(json.dumps(json_dict["e-agents"]))
f.close()

# generate region file, list of transitions to remove, and extra ltl to add to spec
proc = subprocess.Popen(' '.join(["python", LTLMoP_dir+"src/lib/dubins_car/autogenRegion.py", json_file_rnd, region_output_filename]), \
                         shell=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE)
proc.wait()
ltlmop_logger.log(4,'-- Created region file')

#load region file
proj.rfi = regions.RegionFileInterface()
proj.rfi.readFile(region_output_filename)
ltlmop_logger.log(4,'-- Loaded region file')
#proj.loadRegionFile(decomposed=True)

##########################
#### create spec file ####
##########################
# fill in e-agents
#agent_prop_list = []
#for agent_dict in json_dict["e-agents"]:
#    proj.enabled_sensors.append(agent_dict["name"]+'InTheWay')
#    proj.all_sensors.append(agent_dict["name"]+'InTheWay')
#    agent_prop_list.append(agent_dict["name"]+'InTheWay')
agent_prop_list = ['agentAtNextIntersection','agentInSameLaneInFrontOfMe']
proj.enabled_sensors.extend(agent_prop_list)
proj.all_sensors.extend(agent_prop_list)

spec_list = []
# Robot starts in ???
spec_list.append("Robot starts in "+ " or ".join(start_list))

# first check if there's agents
if agent_prop_list:
    # infinitely often not agents in the way
    spec_list.append("infinitely often "+" and ".join(["not "+x for x in agent_prop_list]))

    # stay in place
    spec_list.append("if you are sensing ("+" or ".join(agent_prop_list)+") then stay there")

    # visit goals
    for goal in goal_list:
        spec_list.append("if you are not sensing ("+" or ".join(agent_prop_list)+") then visit "+ goal)
else:
    spec_list.append("visit "+goal)

proj.specText += "\n".join(spec_list)
proj.writeSpecFile()
ltlmop_logger.log(4,'-- Wrote spec file')

#########################
### create SMV file #####
#########################
compiler = specCompiler.SpecCompiler(spec_filename)
compiler._decompose()
spec, tracebackTree, response = compiler._writeLTLFile()
ltlmop_logger.log(4,'-- Wrote LTL file')
compiler._writeSMVFile()
ltlmop_logger.log(4,'-- Wrote SMV file')

##########################
### compile spec file ####
##########################
ltlmop_logger.log(4,'-- Synthesizing spec:')
compiler.prepareSlugsInput()
#realizable,_,log = compiler._synthesize()
#ltlmop_logger.log(4,"".join(['-- Synthesized spec:','realizable' if realizable else 'unrealizable']))
#ltlmop_logger.log(2,log)


##########################
### write config file ####
##########################
# Create blank default config
hsub = handlerSubsystem.HandlerSubsystem(None, proj.project_root)
experiment_config = hsubConfigObjects.ExperimentConfig()
experiment_config.name = "ros_dubins_car"
experiment_config.file_name = proj.project_root+'configs/ros_dubins_car.config'

# assuming the file doesn't exist and create it
if not os.path.exists(proj.project_root+"configs/"):
    os.makedirs(proj.project_root+"configs/")
with open(experiment_config.file_name,'w+') as f:
    pass
f.closed

# ---------- #
# load robot #
# ---------- #
robot_config = hsubConfigObjects.RobotConfig()
try:
    robot_config.fromData({'RobotName':['ros_new'],
                          'Type':['ros_new'],
                          'CalibrationMatrix':'[[{scale}, 0, {offset_x}],\
                                                 [0, {scale}, {offset_y}],\
                                                 [0, 0, 1]]'.format(scale=scale, offset_x=offset[0],offset_y=offset[1]),
                          'InitHandler': ['ros_new.RosInitHandler(init_region="{start_region}",\
                                        robotPixelWidth=200,robotPhysicalWidth=0.2)'.format(start_region=start_list[0])]}, hsub)

except handlerTemplates.LoadingError, msg:
    ltlmop_logger.error(msg)
robot_config.fromFile(LTLMoP_dir+"src/lib/handlers/ros_new/ros_new.robot", hsub)

experiment_config.robots = [robot_config]
experiment_config.main_robot = "ros_new"
hsub.configs.append(experiment_config)
hsub.executing_config = experiment_config #set executing config

# ------------------------- #
# load share handlers only  #
# ------------------------- #
hsub.handler_configs['share'] = {handlerTemplates.SensorHandler:[]}
hsub.handler_configs['share'][handlerTemplates.SensorHandler].append(
    hsub.loadHandler('share', handlerTemplates.SensorHandler, 'DummySensorHandler'))
hsub.handler_configs['ros_new'] = {handlerTemplates.SensorHandler:[]}
hsub.handler_configs['ros_new'][handlerTemplates.SensorHandler].append(
    hsub.loadHandler('ros_new', handlerTemplates.SensorHandler, 'RosSensorHandler'))

# now add in mappings
prop_mapping = {}
# sensor region
for x in proj.rfi.regions:
    if x.name != "boundary" and not x.isObstacle:
        m = copy.deepcopy(hsub.handler_configs["share"][handlerTemplates.SensorHandler][0].getMethodByName("inRegion"))
        para = m.getParaByName("regionName")
        para.setValue(x.name)
        para = m.getParaByName("radius")
        para.setValue(0.1)
        prop_mapping[x.name+'_rc'] = hsub.method2String(m, "share")
#for p in proj.all_sensors:
#    m = copy.deepcopy(hsub.handler_configs["ros_new"][handlerTemplates.SensorHandler][0].getMethodByName("checkIfAgentAtNextRegion"))
#    para = m.getParaByName("agent")
#    para.setValue(p.replace('InTheWay',''))
#    prop_mapping[p] = hsub.method2String(m, "ros_new")

# agentAtNextIntersection
m = copy.deepcopy(hsub.handler_configs["ros_new"][handlerTemplates.SensorHandler][0].getMethodByName("checkIfAgentAtIntsection"))
para = m.getParaByName("json_file_egents")
para.setValue(json_file_egents)
prop_mapping["agentAtNextIntersection"] = hsub.method2String(m, "ros_new")

# agentInSameLaneInFrontOfMe
m = copy.deepcopy(hsub.handler_configs["ros_new"][handlerTemplates.SensorHandler][0].getMethodByName("checkIfAgentInSameLane"))
para = m.getParaByName("json_file_egents")
para.setValue(json_file_egents)
prop_mapping["agentInSameLaneInFrontOfMe"] = hsub.method2String(m, "ros_new")


experiment_config.normalizePropMapping(prop_mapping)

# save file
hsub.saveAllConfigFiles()
ltlmop_logger.log(4,"-- Created config file")
proj.current_config = experiment_config.name
proj.writeSpecFile()
ltlmop_logger.log(4,"-- Updated spec file again.")

####################
## start execute ###
####################
ltlmop_logger.log(4,"-- Starting execution.")
elapsed_time = time.time() - start_time
ltlmop_logger.log(4,"== Total time: {elapsed_time}s ==".format(elapsed_time=elapsed_time))

subprocess.Popen([sys.executable, "-u", "-m", "lib.execute", "-a", proj.getStrategyFilename(), "-s", proj.getFilenamePrefix() + ".spec"])

