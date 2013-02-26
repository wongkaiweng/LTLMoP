#!/usr/bin/env python
"""
=========================================================
FooReadySensor.py - Real Pioneer Robot Sensor Handler
=========================================================
"""

class sensorHandler:
    def __init__(self, proj, shared_data):
        """
        Real Pioneer Robot Sensor handler
        """

        self.robocomm = shared_data['robocomm']


    ###################################
    ### Available sensor functions: ###
    ###################################
    def riceReady(self):
        """
        return a flag sent from Pioneer telling whether there is a dynamic obstacle nearby or not
        """
        if self.robocomm.listener.rice ==True:
            print "rice is true now"
        return self.robocomm.listener.rice

    def sakeReady(self):
        """
        return a flag sent from Pioneer telling whether there is a dynamic obstacle nearby or not
        """
        if self.robocomm.listener.sake ==True:
            print "rice is true now"
        return self.robocomm.listener.sake

