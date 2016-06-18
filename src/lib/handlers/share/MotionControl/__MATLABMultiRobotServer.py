import time
import os
import sys
# Climb the tree to find out where we are
p = os.path.abspath(__file__)
t = ""
while t != "src":
    (p, t) = os.path.split(p)
    if p == "":
        print "I have no idea where I am; this is ridiculous"
        sys.exit(1)
sys.path.append(os.path.join(p,"src","lib"))
import logging
from collections import OrderedDict
import threading
import argparse
import matlab.engine

import regions
import project
import handlerSubsystem
import __MATLABPythonInterface_direct as MATLABPythonInterface


class MATLABMuliRobotServer(object):
    def __init__(self, project_root, project_basename, no_of_robots):
        """
        project_root: full path to example foler
        project_basename: spec file name without .spec from project_root
        """

        region_output_filename = project_root+project_basename+"_decomposed.regions"
        self.rfi = regions.RegionFileInterface()
        self.rfi.readFile(region_output_filename)
        logging.info('-- Loaded region file')

        logging.info('-- Start MATLAB Session')
        #logging.warning(self.proj.rfi.regions)
        #logging.warning(self.hsub.coordmap_map2lab)
        self.session = MATLABPythonInterface.initializeMATLABPythonCommunication(self.rfi.regions, None)
        self.session.eval("matlab.engine.shareEngine('MATLAB_Nora')",nargout=0)


    def init_execution(self, robot_name, pose, next_region_idx):
        #pose  = {'rob1':[-1 ,.5],'rob2':[1,1],'rob3':[3.5 , -1]}
        #next_regIndices = {'rob1': 2,'rob2':3,'rob3':3}
        with self._lock:
            self.robot_pose[robot_name] = pose
            self.next_region_indice[robot_name] = next_region_idx
            logging.debug("INIT: self.robot_pose: {robot_pose}".format(robot_pose=self.robot_pose))
            logging.debug("INIT: self.next_region_indice: {next_region_indice}".format(next_region_indice=self.next_region_indice))

        print "exec:" + str(self.session.workspace)

    def get_velocity(self, robot_name, pose, next_region_idx):
        with self._lock:
            # check if region idx is the same, if not then replan
            if next_region_idx != self.next_region_indice[robot_name]:
                replan = [True]
            else:
                replan = [False]

            self.robot_pose[robot_name] = pose
            self.next_region_indice[robot_name] = next_region_idx

            vx, vy, regionChanges, currentLoc = MATLABPythonInterface.getMATLABVelocity(self.session,\
                        self.robot_pose, self.next_region_indice, replan)

            return vx, vy, regionChanges, currentLoc


def main():
    # get arguments
    parser = argparse.ArgumentParser(description="MATLAB Server.")
    parser.add_argument('--region_file_directory', type=str, help='Directory to region file with last slash')
    parser.add_argument('--region_filename', type=str, help='name of region file')
    parser.add_argument('--no_of_robots', type=int, help='total number of robots')
    args = parser.parse_args()

    # get arguments
    if args.region_file_directory:
        region_file_directory = args.region_file_directory
    else:
        region_file_directory = "/home/catherine/LTLMoP/src/examples/multiRobot/three_robots_three_items/"

    if args.region_filename:
        region_filename = args.region_filename
    else:
        region_filename = "three_robots_three_items"

    if args.no_of_robots:
        no_of_robots = args.no_of_robots
    else:
        no_of_robots = 3


    a = MATLABMuliRobotServer(region_file_directory, region_filename, no_of_robots)


    try:
        logging.info("Running MATLAB...")
        while True:
            time.sleep(1)
    except (KeyboardInterrupt, SystemExit):
        print "we are exiting..."

    print "Shutting down"

main()
