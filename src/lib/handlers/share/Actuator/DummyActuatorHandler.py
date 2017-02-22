#!/usr/bin/env python
"""
=========================================
dummyActuator.py - Dummy Actuator Handler
=========================================

Does nothing more than print the actuator name and state; for testing purposes.
"""

import subprocess, os, time, socket
import sys
import random

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

import Tkinter as tk
from PIL import Image, ImageTk
import threading

import lib.handlers.share.Actuator.__light_switches as light_switches
import lib.handlers.handlerTemplates as handlerTemplates

class DummyActuatorHandler(handlerTemplates.ActuatorHandler):
    def __init__(self, executor, shared_data):
        if executor:
            self.proj = executor.proj
        self.p_gui = None

        self.tkRootThread = None # store tk root
        self.tkWindows = {} # dict of windows derived from root
        self.tkButtonWindow = None
        self.buttonPressCompletionStatus = {}
        self.imageDisplayCompletionStatus = {}

    def _stop(self):
        if self.p_gui is not None:
            print >>sys.__stderr__, "(ACT) Killing dummyactuator GUI..."
            try:
                self.p_gui.stdin.write(":QUIT\n")
                self.p_gui.stdin.close()
            except IOError:
                # Probably already closed by user
                pass

        # destroy completion windows
        for imageWindow in self.tkWindows.values():
            imageWindow.quit = True

        if self.tkRootThread:
            print >>sys.__stderr__, "(ACT) Killing tkRoot..."
            self.tkRootThread.callback()

    def setActuator(self, name, actuatorVal,initial):
        """
        Pretends to set actuator of name ``name`` to be in state ``val`` (bool).

        name (string): Name of the actuator
        """

        if initial:
            if self.p_gui is None:
                # Prepare to receive initialization signal
                host = 'localhost'
                port = random.randint(10000, 65535)  #port = 23559
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
                self.p_gui = subprocess.Popen([sys.executable, "-u", os.path.join(self.proj.ltlmop_root,"lib","handlers","share","Actuator","_ActuatorHandler.py"),str(port)], stderr=subprocess.PIPE, stdin=subprocess.PIPE)

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
        else:
            if actuatorVal:
                time.sleep(0.1)  # Fake some time lag for the actuator to enable

            self.p_gui.stdin.write("{},{}\n".format(name,int(actuatorVal)))

            print "(ACT) Actuator %s is now %s!" % tuple(map(str, (name, actuatorVal)))

    def imageDisplay(self, actuatorName, trueImage, falseImage, actuatorVal, initial):
        """
        Diplay two different images depending on actuator value.

        actuatorName (string): Name of the actuator whose state is interested.
        trueImage (string): Path to image. Display image when actuator is true.
        falseImage (string): Path to image. Display image when actuator is false.
        """
        if initial:
            ltlmop_logger.log(4,'we are initializing...')

            # first check if the tk root has started
            if not self.tkRootThread:
                self.tkRootThread = _tkRoot()
                time.sleep(2)

            # initialize window
            self.imageDisplayCompletionStatus[actuatorName] = False
            self.tkWindows[actuatorName] = _tkImageWindow(self.tkRootThread.root, actuatorVal, trueImage, falseImage, \
                                                          self.imageDisplayCompletionStatus, actuatorName)
        else:
            # update actuator value
            self.tkWindows[actuatorName].update_actuatorVal(actuatorVal)
            ltlmop_logger.debug(self.imageDisplayCompletionStatus)


    def buttonPress(self, actuatorName, actuatorVal, initial):
        """
        Press button. push button down when true and release button when false

        actuatorName (string): Name of the actuator whose state is interested.
        """
        if initial:
            ltlmop_logger.log(4,'we are initializing...')

            # first check if the tk root has started
            if not self.tkRootThread:
                self.tkRootThread = _tkRoot()
                # start tk window
                self.tkButtonWindow = light_switches._tkButtonWindow(self.tkRootThread.root, self.buttonPressCompletionStatus)
                time.sleep(2)

            # add button
            self.tkButtonWindow.add_button(actuatorName, actuatorVal)

        else:
            # update actuator value
            self.tkButtonWindow.update_actuatorVal(actuatorName, actuatorVal)
            ltlmop_logger.debug('Completion of button press: {0}'.format(self.buttonPressCompletionStatus[actuatorName]))


class _tkImageWindow(object):
    def __init__(self, master, actuatorVal, trueImage, falseImage, imageDisplayCompletionStatusDict, actuatorName):
        """
        This object opens a window from master.
        """
        self.master = master
        self.trueImage = trueImage
        self.falseImage = falseImage
        self.actuatorVal = actuatorVal
        self.imageDisplayCompletionStatusDict = imageDisplayCompletionStatusDict
        self.actuatorName = actuatorName
        self.quit = False

        # set up initial display
        #self.top = self.master
        #ltlmop_logger.log(4,self.master)
        self.top = tk.Toplevel(self.master)
        img = ImageTk.PhotoImage(Image.open(self.trueImage if self.actuatorVal else self.falseImage))
        self.panel = tk.Label(self.top, image=img)
        self.panel.pack(side = "bottom", fill = "both", expand = "yes")
        ltlmop_logger.debug('we have initialized')

        # for fullscreen mode
        self.state= False
        self.top.bind("<F11>", self.toggle_fullscreen)
        self.top.bind("<Escape>", self.end_fullscreen)
        # update every 0.5s
        if not self.quit:
            self.top.after(500, self.update_image)

    def update_actuatorVal(self, actuatorVal):
        self.actuatorVal = actuatorVal

    def update_image(self):
        # update based on actuatorVal
        #ltlmop_logger.log(2, 'we did update image')
        img = ImageTk.PhotoImage(Image.open(self.trueImage if self.actuatorVal else self.falseImage))
        self.panel.configure(image=img)
        self.panel.image = img

        #note: we need to update completion sensors here too!
        self.imageDisplayCompletionStatusDict[self.actuatorName] = True if self.actuatorVal else False
        if not self.quit:
            self.top.after(500, self.update_image)

    def toggle_fullscreen(self, event=None):
        self.state = not self.state  # Just toggling the boolean
        self.top.attributes("-fullscreen", self.state)
        return "break"

    def end_fullscreen(self, event=None):
        self.state = False
        self.top.attributes("-fullscreen", False)
        return "break"

class _tkRoot(threading.Thread):
    def __init__(self):
        """
        This object starts the root tk Thread.
        """
        threading.Thread.__init__(self)
        self.root = None
        self.daemon = True
        self.start()

    def callback(self):
        #self.root.destroy()
        self.root.quit()
        ltlmop_logger.info("We did destory tkRoot")

    def run(self):
        self.root = tk.Tk()
        self.root.withdraw()
        self.root.mainloop()

if __name__ == "__main__":
    # a = DummyActuatorHandler(None, None)

    # #falseImage = '/home/catherine/Desktop/test.jpg'
    # falseImage ='/home/catherine/LTLMoP/src/examples/_single_robot_example/kitchen/pic_map/mapDashed_with_door_nao-01.png'
    # trueImage  ='/home/catherine/LTLMoP/src/examples/_single_robot_example/kitchen/pic_map/mapDashed_without_door_nao-01.png'

    # a.imageDisplay("updateBackground", falseImage, trueImage, False, initial=True)
    # time.sleep(2)
    # a.imageDisplay("updateBackground", falseImage, trueImage, True, initial=False)
    # time.sleep(2)
    # a.imageDisplay("updateBackground", falseImage, trueImage, False, initial=False)
    # time.sleep(2)
    # a._stop()

    a = DummyActuatorHandler(None, None)
    #tkRootThread = _tkRoot()
    #a =_tkButtonWindow(tkRootThread.root)

    a.buttonPress('button_1', False, initial=True)
    time.sleep(2)
    a.buttonPress('button_1', True, initial=False)
    time.sleep(2)
    a.buttonPress('button_1', False, initial=False)
    time.sleep(2)

    # button 2
    a.buttonPress('button_2', False, initial=True)
    time.sleep(2)
    a.buttonPress('button_2', True, initial=False)
    time.sleep(2)
    a.buttonPress('button_2', False, initial=False)
    time.sleep(2)
    a._stop()