#!/usr/bin/env python
"""
====================================================
BaxterSensorHandler.py - Sensor handler for the Baxter
====================================================
"""

# logger for ltlmop
import logging
ltlmop_logger = logging.getLogger('ltlmop_logger')

import lib.handlers.handlerTemplates as handlerTemplates

class BaxterSensorHandler(handlerTemplates.SensorHandler):
    def __init__(self, executor, shared_data):
        self.baxterInitHandler = shared_data['BAXTER_INIT_HANDLER']

    ###################################
    ### Available sensor functions: ###
    ###################################
    def play_traj_completion(self, actuatorName, initial=False):
        """
        This function works in pair with play_traj in BaxterActuatorHandler.py
        And must use the same actuatorName to refer to the same action played.

        actuatorName (string): name of the actuation
        """
        if initial:
            if actuatorName not in self.baxterInitHandler.play_traj_status_sen.keys():
                ltlmop_logger.error('Actuation:' + actuatorName + "is not initialized.")
                return False
            return True
        else:
            # check if we have such an action to start with
            if actuatorName not in self.baxterInitHandler.play_traj_status_sen.keys():
                ltlmop_logger.error('Actuation:' + actuatorName + "is not initialized.")
                return False
            else:
                # return status
                #if self.baxterInitHandler.play_traj_status_sen[actuatorName]:
                    #ltlmop_logger.debug('completed actuator:' + actuatorName)
                return self.baxterInitHandler.play_traj_status_sen[actuatorName]

