import threading
import Tkinter as tk
from PIL import Image, ImageTk

import time
import logging
import socket

h = logging.StreamHandler()
ltlmop_logger = logging.getLogger('ltlmop_logger')
if not ltlmop_logger.handlers:
    ltlmop_logger.addHandler(h)
ltlmop_logger.setLevel(logging.DEBUG)


class socketBroadCast(object):
    def __init__(self, port, defaultValue):
        """
        set up socket broadcast with the current port.
        Provide default value.
        """
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.port = port
        self.value = defaultValue

        # initial broadcast
        self._broadcastBool(self.value)

    def _broadcastBool(self, value):
        """
        send either True or False.
        port
        """
        self.s.sendto(str(value),('255.255.255.255',self.port))

class _tkImageWindow(object):
    def __init__(self, master, actuatorVal, trueImage, falseImage, brokenImage, imageDisplayCompletionStatusDict, actuatorName):
        """
        This object opens a window from master.
        """
        self.master = master
        self.trueImage = trueImage
        self.falseImage = falseImage
        self.brokenImage = brokenImage
        self.actuatorVal = actuatorVal
        self.trueKey = "<F1>"
        self.falseKey = "<Return>"
        self.brokenKey = "<F2>"
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
        self.panel.configure(image=img)
        self.panel.image = img
        ltlmop_logger.debug('we have initialized')

        # for fullscreen mode
        self.state= False
        self.top.bind("<F11>", self.toggle_fullscreen)
        self.top.bind("<Escape>", self.end_fullscreen)
        self.top.bind(self.trueKey, lambda event, arg=self.trueImage: self.callback(event, arg))
        self.top.bind(self.falseKey, lambda event, arg=self.falseImage: self.callback(event, arg))
        self.top.bind(self.brokenKey, lambda event, arg=self.brokenImage: self.callback(event, arg))

        # set up socket
        self.openSocket = socketBroadCast(12345, False)
        self.brokenSocket = socketBroadCast(12346, False)

        # update every 0.5s
        #if not self.quit:
        #    self.top.after(500, self.update_image)

    def callback(self, e, imagePath):
        img = ImageTk.PhotoImage(Image.open(imagePath))
        self.panel.configure(image=img)
        self.panel.image = img

        if "<"+e.keysym+">" == self.trueKey:
            ltlmop_logger.debug("true image")
            # update bool broadcase
            self.openSocket._broadcastBool(True)
            self.brokenSocket._broadcastBool(False)

        elif "<"+e.keysym+">" == self.falseKey:
            ltlmop_logger.debug("false image")
            # update bool broadcase
            self.openSocket._broadcastBool(False)
            self.brokenSocket._broadcastBool(False)

        elif "<"+e.keysym+">" == self.brokenKey:
            ltlmop_logger.debug("brokenImage")
            # update bool broadcase
            self.openSocket._broadcastBool(False)
            self.brokenSocket._broadcastBool(True)

        else:
            ltlmop_logger.debug('not sure why we are here!')

    def update_actuatorVal(self, actuatorVal):
        self.actuatorVal = actuatorVal

    def update_image(self):
        # update based on actuatorVal
        #ltlmop_logger.log(2, 'we did update image')
        img = ImageTk.PhotoImage(Image.open(self.trueImage if self.actuatorVal else self.falseImage))
        self.panel.configure(image=img)
        self.panel.image = img

        #note: we need to update completion sensors here too!
        #self.imageDisplayCompletionStatusDict[self.actuatorName] = True if self.actuatorVal else False
        #if not self.quit:
        #    self.top.after(500, self.update_image)

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
    #a = DummyActuatorHandler(None, None)
    tkRootThread = _tkRoot()
    #falseImage = '/home/catherine/Desktop/test.jpg'
    falseImage ='/home/catherine/LTLMoP/src/examples/_single_robot_example/kitchen/pic_map/mapDashed_with_door_nao-01.png'
    trueImage  ='/home/catherine/LTLMoP/src/examples/_single_robot_example/kitchen/pic_map/mapDashed_without_door_nao-01.png'
    brokenImage='/home/catherine/LTLMoP/src/examples/_single_robot_example/kitchen/pic_map/mapDashed_repairing-01.png'
    actuatorName = 'myActuator'
    actuatorVal = False
    a =_tkImageWindow(tkRootThread.root, actuatorVal, trueImage, falseImage, \
          brokenImage, None, actuatorName)

    while True:
        time.sleep(0.1)
    #a.imageDisplay("updateBackground", falseImage, trueImage, False, initial=True)
    #time.sleep(2)
    #a.imageDisplay("updateBackground", falseImage, trueImage, True, initial=False)
    #time.sleep(2)
    #a.imageDisplay("updateBackground", falseImage, trueImage, False, initial=False)
    #time.sleep(2)
    #a._stop()