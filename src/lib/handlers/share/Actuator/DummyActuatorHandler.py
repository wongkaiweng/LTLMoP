#!/usr/bin/env python
"""
=========================================
dummyActuator.py - Dummy Actuator Handler
=========================================

Does nothing more than print the actuator name and state; for testing purposes.
"""

import subprocess, os, time, socket
import sys
import threading

import lib.handlers.handlerTemplates as handlerTemplates

class DummyActuatorHandler(handlerTemplates.ActuatorHandler):
    def __init__(self, executor, shared_data):
        self.proj = executor.proj
        self.p_gui = None

        self.thread = {} # a dictionary that holds the thread for each handler method config (hmc)

    def _stop(self):
        if self.p_gui is not None:
            print >>sys.__stderr__, "(ACT) Killing dummyactuator GUI..."
            try:
                self.p_gui.stdin.write(":QUIT\n")
                self.p_gui.stdin.close()
            except IOError:
                # Probably already closed by user
                pass

    def setActuator(self, name, delay, actuatorVal, initial, hmc_ref):
        """
        Pretends to set actuator of name ``name`` to be in state ``val`` (bool).

        name (string): Name of the actuator
        delay (float): Time in second needed to change the actuator (default=0.0,min=0.0,max=10.0)
        """

        if initial:
            if self.p_gui is None:
                # Prepare to receive initialization signal
                host = 'localhost'
                port = 23559
                buf = 1024
                addr = (host,port)

                UDPSock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
                UDPSock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                try:
                    UDPSock.bind(addr)
                except:
                    print "ERROR: Cannot bind to port.  Try killing all Python processes and trying again."
                    return

                # Create a subprocess
                print "(ACT) Starting actuatorHandler window..."
                self.p_gui = subprocess.Popen([sys.executable, "-u", os.path.join(self.proj.ltlmop_root,"lib","handlers","share","Actuator","_ActuatorHandler.py")], stderr=subprocess.PIPE, stdin=subprocess.PIPE)

                data = ''
                while "Hello!" not in data:
                    # Wait for and receive a message from the subwindow
                    try:
                        data,addrFrom = UDPSock.recvfrom(1024)
                    except socket.timeout:
                        print "Waiting for GUI..."
                        continue

                UDPSock.close()

            self.p_gui.stdin.write(name + ",init\n")

            # set the thread for this hmc
            self.thread[hmc_ref] = threading.Thread()
        else:
            if (not self.thread[hmc_ref].is_alive()) and (hmc_ref.actuator_state != actuatorVal):
                # no thread is running
                # renew the thread
                self.thread[hmc_ref] = threading.Timer(delay, hmc_ref.updateActuatorState, [actuatorVal])
                self.thread[hmc_ref].start()
            elif self.thread[hmc_ref].is_alive() and (self.actuator_status == actuatorVal):
                # a thread is running
                # we need to stop the thread
                self.thread[hmc_ref].cancel()

            self.p_gui.stdin.write("{},{}\n".format(name,int(actuatorVal)))

            print "(ACT) Actuator %s is now %s!" % tuple(map(str, (name, actuatorVal)))

