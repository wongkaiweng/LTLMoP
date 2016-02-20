#!/usr/bin/env python
"""
=================================================
BaxterInitHandler.py - Baxter Initialization Handler
=================================================

Initialize the proxies to access baxter modules
"""

import rospy
import sys, os

import baxter_interface
from baxter_interface import CHECK_VERSION

# Climb the tree to find out where we are
p = os.path.abspath(__file__)
t = ""
while t != "src":
    (p, t) = os.path.split(p)
    if p == "":
        print "I have no idea where I am; this is ridiculous"
        sys.exit(1)

sys.path.append(os.path.join(p,"src","lib"))

# logger for ltlmop
import logging
ltlmop_logger = logging.getLogger('ltlmop_logger')

import lib.handlers.handlerTemplates as handlerTemplates

class BaxterInitHandler(handlerTemplates.InitHandler):
    def __init__(self, executor):
        """
        Initialization handler for baxter robot.

        Note that currently, before running LTLMoP, you should have run ./baxter.sh in the same
        shell before hand.
        """

        """
        First we need to make sure the robot is enabled.
        """
        ltlmop_logger.info("Initializing baxter everything node... ")
        rospy.init_node("baxter_node")

        ltlmop_logger.info("Getting robot state... ")
        rs = baxter_interface.RobotEnable(CHECK_VERSION)
        ltlmop_logger.info("Enabling robot... ")
        rs.enable()


    def getSharedData(self):
        return {'BAXTER_INIT_HANDLER': self}


if __name__ == "__main__":
    import BaxterActuatorHandler
    import BaxterSensorHandler
    import time
    import sys

    init = BaxterInitHandler(None)
    act = BaxterActuatorHandler.BaxterActuatorHandler(None, init.getSharedData(), 'both', 100, 'position_w_id')
    sen = BaxterSensorHandler.BaxterSensorHandler(None, init.getSharedData())
    act.play_traj('lib/handlers/baxter/actions/pickup_clean.rec', 'pickup', 1, 0, initial=True)
    act.play_traj('lib/handlers/baxter/actions/cleanup_return.rec', 'drop', 1, 0, initial=True)

    stop = False
    # run pickup action
    while not stop:
        try:
            act.play_traj('lib/handlers/baxter/actions/pickup_clean.rec', 'pickup', 1, 1, initial=False)
            while not sen.play_traj_completion('pickup') and not stop:
                try:
                    ##### test this next!!! #####
                    time.sleep(10.0)
                    ltlmop_logger.debug('pausing...')
                    act._pause()
                    time.sleep(3.0)
                #time.sleep(3.0)
                    ltlmop_logger.debug('stopping...')
                    act.play_traj('lib/handlers/baxter/actions/pickup_clean.rec', 'pickup', 1, 0, initial=False)
                    time.sleep(1.0)
                    act._resume()
                    ltlmop_logger.debug('resuming...')
                    ltlmop_logger.info('Action pickup is not completed.')

                    # act.play_traj('lib/handlers/baxter/actions/cleanup_return.rec', 'drop', 1, 1, initial=False)
                    # while not sen.play_traj_completion('drop'):
                    #     time.sleep(0.5)
                    #     ltlmop_logger.info('Action drop is not completed.')
                    # act.play_traj('lib/handlers/baxter/actions/cleanup_return.rec', 'drop', 1, 0, initial=False)
                except KeyboardInterrupt:
                    stop = True
                    sys.exit()

        except KeyboardInterrupt:
            stop = True
            sys.exit()
    rospy.signal_shutdown("Action completed.")
    time.sleep(1)
