import threading
import Tkinter as tk
from PIL import Image, ImageTk

import time
import logging
import socket
import getpass
import string

h = logging.StreamHandler()
ltlmop_logger = logging.getLogger('ltlmop_logger')
if not ltlmop_logger.handlers:
    ltlmop_logger.addHandler(h)
ltlmop_logger.setLevel(logging.DEBUG)

NO_OF_LIGHTS = 3
NO_OF_BUTTONS = 3

BRIGHT_COLOR = 'yellow'
DARK_COLOR = 'black'

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

class _tkButtonWindow(object):
    def __init__(self, master, buttonPressCompletionStatusDict={}):
        """
        This object opens a window from master.
        """
        self.master = master
        self.trueKey = "<F1>"
        self.falseKey = "<Return>"
        self.brokenKey = "<F2>"
        self.buttonPressCompletionStatusDict = buttonPressCompletionStatusDict
        self.actuator_list = []
        self.actuator_dict = {}
        self.quit = False

        # set up initial display
        #self.top = self.master
        #ltlmop_logger.log(4,self.master)
        self.top = tk.Toplevel(self.master)
        self.top.geometry("%dx%d%+d%+d" % (300, 200, 250, 125))
        self.lights_tk_list = []
        self.lights_socket_list = []
        self.button_tk_list = []

        # no of lights
        for idx in range(NO_OF_LIGHTS):
            self.lights_tk_list.append(tk.Label(self.top, text=idx+1, bg=DARK_COLOR, fg='gray'))
            self.lights_tk_list[idx].pack()
            self.lights_tk_list[idx].place(bordermode='outside', relheight=1.0/NO_OF_LIGHTS,\
                                                              relwidth=1.0/NO_OF_LIGHTS-0.1/NO_OF_LIGHTS, \
                                                              relx=0+1.0/NO_OF_LIGHTS*idx)

            # light sockets
            self.lights_socket_list.append(socketBroadCast(12345+idx, False))

        # no of buttons
        for idx in range(NO_OF_BUTTONS):
            self.button_tk_list.append(tk.Button(self.top, text=list(string.ascii_uppercase)[idx], command=lambda idx=idx: self.button_callback(idx)))
            self.button_tk_list[idx].pack()
            self.button_tk_list[idx].place(bordermode='outside', relheight=1.0/NO_OF_LIGHTS,\
                                                              relwidth=1.0/NO_OF_LIGHTS-0.1/NO_OF_LIGHTS, \
                                                              relx=0+1.0/NO_OF_LIGHTS*idx,\
                                                              rely=0.6)

        ltlmop_logger.debug('we have initialized')

        # for fullscreen mode
        self.state= False
        self.top.bind("<F11>", self.toggle_fullscreen)
        self.top.bind("<Escape>", self.end_fullscreen)
        #self.top.bind(self.trueKey, lambda event, arg=self.trueImage: self.callback(event, arg))
        #self.top.bind(self.falseKey, lambda event, arg=self.falseImage: self.callback(event, arg))
        #self.top.bind(self.brokenKey, lambda event, arg=self.brokenImage: self.callback(event, arg))

    def add_button(self, actuatorName, initial=False):
        self.actuator_list.append(actuatorName)
        self.actuator_list = sorted(self.actuator_list) # sort the list
        self.actuator_dict[actuatorName] = initial
        self.buttonPressCompletionStatusDict[actuatorName] = False

    def button_callback(self, idx):
        # button A (toggle 1, 3)
        if idx == 0:
            self.button_tk_list[idx].config(relief=tk.SUNKEN if self.button_tk_list[idx]["relief"] != tk.SUNKEN else tk.RAISED)
            self.lights_tk_list[0]["bg"] = DARK_COLOR if self.lights_tk_list[0]["bg"] != DARK_COLOR else BRIGHT_COLOR
            self.lights_tk_list[2]["bg"] = DARK_COLOR if self.lights_tk_list[2]["bg"] != DARK_COLOR else BRIGHT_COLOR

        # button B (toggle 2)
        elif idx == 1:
            self.button_tk_list[idx].config(relief=tk.SUNKEN if self.button_tk_list[idx]["relief"] != tk.SUNKEN else tk.RAISED)
            self.lights_tk_list[1]["bg"] = DARK_COLOR if self.lights_tk_list[1]["bg"] != DARK_COLOR else BRIGHT_COLOR

        # button C (toggle 2, 3)
        elif idx == 2:
            self.button_tk_list[idx].config(relief=tk.SUNKEN if self.button_tk_list[idx]["relief"] != tk.SUNKEN else tk.RAISED)
            self.lights_tk_list[1]["bg"] = DARK_COLOR if self.lights_tk_list[1]["bg"] != DARK_COLOR else BRIGHT_COLOR
            self.lights_tk_list[2]["bg"] = DARK_COLOR if self.lights_tk_list[2]["bg"] != DARK_COLOR else BRIGHT_COLOR

        for list_idx, lights_socket_obj in enumerate(self.lights_socket_list):
            lights_socket_obj._broadcastBool(True if self.lights_tk_list[list_idx]["bg"]== BRIGHT_COLOR else False)

        # update actuator and completion status
        self.actuator_dict[self.actuator_list[idx]] = True if self.button_tk_list[idx]["relief"] != tk.RAISED else False
        self.buttonPressCompletionStatusDict[self.actuator_list[idx]] = True if self.button_tk_list[idx]["relief"] != tk.RAISED else False


    def update_actuatorVal(self, actuatorName, actuatorVal):
        # true but raised
        # false but sunken
        if actuatorVal and self.button_tk_list[self.actuator_list.index(actuatorName)]["relief"] != tk.SUNKEN or \
           not actuatorVal and self.button_tk_list[self.actuator_list.index(actuatorName)]["relief"] != tk.RAISED:
            # then call button callback
            self.button_callback(self.actuator_list.index(actuatorName))


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
        f1= tk.Frame(self.root, height=200, width=200)
        f1.pack()
        b = tk.Button(f1, text='asldkf')
        b.pack()
        self.root.withdraw()
        self.root.mainloop()

if __name__ == "__main__":
    #a = DummyActuatorHandler(None, None)
    tkRootThread = _tkRoot()
    a =_tkButtonWindow(tkRootThread.root)

    a.add_button('button_A')
    a.add_button('button_B')
    a.add_button('button_C')

    while True:
        time.sleep(0.1)

    #a.buttonPress('button_1', False, initial=True)
    #time.sleep(2)
    #a.buttonPress('button_1', True, initial=True)
    #time.sleep(2)
    #a.buttonPress('button_1', False, initial=True)
    #time.sleep(2)
    #a._stop()
    #a.imageDisplay("updateBackground", falseImage, trueImage, False, initial=True)
    #time.sleep(2)
    #a.imageDisplay("updateBackground", falseImage, trueImage, True, initial=False)
    #time.sleep(2)
    #a.imageDisplay("updateBackground", falseImage, trueImage, False, initial=False)
    #time.sleep(2)
    #a._stop()