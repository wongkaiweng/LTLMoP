#!/usr/bin/env python
"""
=================================================
FoodReadyInit.py - Real Pioneer Robot Initialization Handler
=================================================

Initialize the sockets to communicate with C# on the Pioneer
"""

import time
from socket import *
from struct import pack, unpack
from threading import Thread, Lock, Event
from numpy import *

class initHandler:

    def __init__(self, proj,NetduinoIP="192.168.5.100",ListenerPort = 12000):
        """
        Init Handler for pioneer real robot.

        NetduinoIP (string)     : Local IP of Netduino       (default="192.168.5.100")
        ListenerPort (int)      : Listering Port of Pioneer  (default=12000)

        """


        # Get connection settings from robot configuration file
        ipIn = NetduinoIP                  # IP address (string)
        portIn = ListenerPort           # Port (number)
        print "should print"
        try:
            # Create proxies to access modules
            self.robocomm = _RobotCommunicator(ipIn,portIn)
            self.robocomm.start()
            time.sleep(1)   # Give communicator time to start and receive first data
        except RuntimeError:
            print "(INIT) ERROR: Cannot connect to the robot."
            exit(-1)

    def getSharedData(self):
        # Return dictionary of module proxies for other handlers to use
        return {'robocomm':self.robocomm}

class _RobotCommunicator:
    """
    Class used to communicate from LTLMoP in Python to C# on Pioneers or
    Segways. Is designed to be consistent with LTLMoPSerializer C# class.
    """

    DEFAULT_BUFFER_SIZE = 10240

    # Constructor
    def __init__(self,ipIn,portIn):
        # Communication parameters
        self.listenerIP = ipIn
        self.listenerPort = portIn

        self.buffer = _RobotCommunicator.DEFAULT_BUFFER_SIZE

        # Communication threads
        self.listener = _RobotListener(self.listenerIP,self.listenerPort,
                                      self.buffer)

    def start(self):
        """
        Open sockets for communication.
        """
        self.listener.start()
        # No need to start broadcaster, it just sends when necessary

    def stop(self):
        """
        Close sockets and prevent further communication.
        """
        self.listener.stop()

class _RobotListener(Thread):
    """
    Class used to communicate from a robot to LTLMoP. Is designed to be
    consistent with LTLMoPSerializer C# class.
    """

    # Constructor
    def __init__(self,ipAddress,port,bufferSize):
        # Superclass constructor
        super(_RobotListener,self).__init__()

        # Communication parameters
        self.addr = (ipAddress,port)
        self.buffer = bufferSize
        self.tcpSock = socket(AF_INET,SOCK_STREAM)
        self.lock = Lock()
        self.close = Event()

        # Data fields
        self.rice = False
        self.sake = False


    # Start communication and receive messages
    def run(self):  #CHANGED FROM run to start
        """
        Open the socket to start UDP communication.
        Receive and process messages.
        """

        # Open socket for communication
        #self.tcpSock.bind(self.addr)
        try:
            self.tcpSock.connect((self.addr))
        except Exception:
            print "Exception caught for connecting to Netduino"
        # Receive communication until stopped
        print self.addr
        print "bind listen succeed!"
        while not self.close.isSet():
            data = self.tcpSock.recv(self.buffer)
            #print data
            self.lock.acquire()
            self.processData(data)
            self.lock.release()

        # Close socket
        self.tcpSock.close()

    # Stop communication
    def stop(self):
        """
        Close the socket to end UDP communication.
        """
        self.close.set()

    # Deserialize and save data
    def processData(self,data):
        """
        Save the data to the appropriate category for possible extraction.
        """
        if data:
            if (data.find("rice"))!= -1:
                if data.split("rice:")[1] == "T":
                    self.rice = True
                else:
                    self.rice = False
            elif (data.find("sake"))!= -1:
                if data.split("sake:")[1] == "T":
                    self.sake = True
                else:
                    self.sake = False
        else:
            print "No data received."
