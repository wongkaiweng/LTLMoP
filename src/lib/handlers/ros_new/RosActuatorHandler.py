#!/usr/bin/env python
"""
===================================================
rosActuator.py - Actuation Handler for ROS intefarce
===================================================

Control functions using ROS
"""
import lib.handlers.handlerTemplates as handlerTemplates

class RosActuatorHandler(handlerTemplates.ActuatorHandler):
    def __init__(self, executor, shared_data):
        """
        Actuator Handler for ROS type applications
        """
        self.rosInitHandler = shared_data['ROS_INIT_HANDLER']
        self.loco = executor.hsub.getHandlerInstanceByType(handlerTemplates.LocomotionCommandHandler)

    #####################################
    ### Available actuator functions: ###
    #####################################
