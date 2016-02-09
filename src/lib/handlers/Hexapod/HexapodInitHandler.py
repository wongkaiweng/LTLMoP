"""
================================================================================
HexapodInitHandler.py - The Hexapod's Init Handler
================================================================================
"""

import serial
import globalConfig

# logger for ltlmop
import logging
ltlmop_logger = logging.getLogger('ltlmop_logger')

import lib.handlers.handlerTemplates as handlerTemplates

class HexapodInitHandler(handlerTemplates.InitHandler):
    def __init__(self, executor, comPort):
        """
        The init handler of Hexapod

        comPort (string): The comport to connect to (default="COM5")
        """

        self.hexapodSer = None   #serial port for hexapod
        #Defaults
        self.baud = 9600     #set baud rate
        self.timeout = 0.5     #in seconds

        try:
            self.hexapodSer = serial.Serial(port = comPort, baudrate =
                                           self.baud, timeout = self.timeout)

            # set robot to neutral position
            self.hexapodSer.write("b")
            self.hexapodSer.flush()
            # Wait for "setup complete" (q)
            init_response = self.hexapodSer.read()

            while init_response == '':
                init_response = self.hexapodSer.read()
        except:
            ltlmop_logger.exception("Couldn't connect to Hexapod")
            exit(-1)

    def _stop(self):
        ltlmop_logger.info("Shutting down serial port!")
        self.hexapodSer.close()

    def getSharedData(self):
        return {"hexapodSer":self.hexapodSer}

if __name__ == "__main__":
    h = HexapodInitHandler(None,'COM8')
