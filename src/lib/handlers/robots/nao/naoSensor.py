#!/usr/bin/env python
"""
====================================================
naoSensors.py - Sensor handler for the Aldebaran Nao
====================================================
"""

import sys, os
import math

# Climb the tree to find out where we are
p = os.path.abspath(__file__)
t = ""
while t != "src":
    (p, t) = os.path.split(p)
    if p == "":
        print "I have no idea where I am; this is ridiculous"
        sys.exit(1)

sys.path.append(os.path.join(p,"src","lib"))

import handlers.pose._pyvicon as _pyvicon

class naoSensorHandler:
    def __init__(self, proj, shared_data):
        self.naoInitHandler = shared_data['NAO_INIT_HANDLER']

        self.sttProxy = None
        self.sttVocabulary = []
        self.sttVocabCounter = 0

        self.faceProxy = None
        self.memProxy = None
        self.sttProxy = None
        self.ldmProxy = None
        self.soundProxy = None
        self.proj = proj

    ###################################
    ### Available sensor functions: ###
    ###################################
    def seeLandMark(self,landMark_id,initial=False):
        """
        Use Nao's landmark recognition system to detect radial bar code landmark.
        For info about avaible bar code, refer to http://www.aldebaran-robotics.com/documentation/naoqi/vision/allandmarkdetection.html#allandmarkdetection

        landMark_id (int): The id number of bar code to detect
        """
        if initial:

            # initialize landmark detection
            if self.ldmProxy == None:
                self.ldmProxy = self.naoInitHandler.createProxy('ALLandMarkDetection')
            if self.memProxy is None:
                self.memProxy = self.naoInitHandler.createProxy('ALMemory')

            ### Initialize land Mark tracking
            subs = [x[0] for x in self.ldmProxy.getSubscribersInfo()]
            # Close any previous subscriptions that might have been hanging open
            if "ltlmop_sensorhandler" in subs:
                self.ldmProxy.unsubscribe("ltlmop_sensorhandler")
            self.ldmProxy.subscribe("ltlmop_sensorhandler", 100, 0.0)
            return True
        else:
            val = self.memProxy.getData("LandmarkDetected",0)

            if(val and isinstance(val, list) and len(val) == 5):
                # We detected naomarks !
                # For each mark, we can read its shape info and ID.

                # Second Field = array of Mark_Info's.
                markInfoArray = val[1]

                try:
                    # Browse the markInfoArray to get info on each detected mark.
                    for markInfo in markInfoArray:

                        # First Field = Shape info.
                        markShapeInfo = markInfo[0]

                        # Second Field = Extra info (ie, mark ID).
                        markExtraInfo = markInfo[1]

                        #print " width %.3f - height %.3f" % (markShapeInfo[3], markShapeInfo[4])

                        #if float(markShapeInfo[3])>0.05 and float(markShapeInfo[4])>0.05:
                        if landMark_id in markExtraInfo:
                            self.proj.executor.postEvent("INFO","See landmark " + str(landMark_id))
                            return True

                except Exception, e:
                    print "Naomarks detected, but it seems getData is invalid. ALValue ="
                    print val
                    print "Error msg %s" % (str(e))
            return False
    """
    def hearAnything(self, threshold ,initial=False):

        See if Nao hears anything.
        
        threshold (float): Minimum acceptable detection confidence (default=0.2,min=0,max=1)

        if initial:
            ### Initialize speech-to-text

            if self.memProxy is None:
                self.memProxy = self.naoInitHandler.createProxy('ALMemory')

            if self.soundProxy is None:
                self.soundProxy = self.naoInitHandler.createProxy('ALSoundDetection')

            return True
         
        else:
            # Check sound state
            sound = self.memProxy.getData("SoundDetected",0)

            # 'SoundDetected' data structure
            #[[index_1, type_1, confidence_1, time_1], ...,[index_n, type_n, confidence_n, time_n]]
            for x in sound:
                print("INFO","sensor.py:" + str(x))
                if x[2] >= threshold:
                    return True
                
            return False
    """                
    def hearWord(self, word, threshold, initial=False):
        """
        Use Nao's speech recognition system to detect a spoken word.

        word (string): The word to detect
        threshold (float): Minimum acceptable detection confidence (default=0.2,min=0,max=1)
        """

        if initial:
            ### Initialize speech-to-text

            if self.memProxy is None:
                self.memProxy = self.naoInitHandler.createProxy('ALMemory')

            if self.sttProxy is None:
                self.sttProxy = self.naoInitHandler.createProxy('ALSpeechRecognition')

            # Close any previous subscriptions that might have been hanging open
            subs = [x[0] for x in self.sttProxy.getSubscribersInfo()]
            if "ltlmop_sensorhandler" in subs:
                self.sttProxy.unsubscribe("ltlmop_sensorhandler")

            self.sttVocabulary += [word]
            self.sttProxy.setWordListAsVocabulary(self.sttVocabulary)

            self.sttProxy.setLanguage("English")
            self.sttProxy.setAudioExpression(False)
            self.sttProxy.setVisualExpression(True)
            self.sttProxy.subscribe("ltlmop_sensorhandler")

            # Reset the speech recognition register manually
            self.memProxy.insertData("WordRecognized", [])

            return True
        else:
            # Check speech recognition state

            wds = self.memProxy.getData("WordRecognized",0)

            # HACK: reset the speech recognition register manually once per vocab-cycle
            self.sttVocabCounter += 1
            if self.sttVocabCounter == len(self.sttVocabulary):
                self.memProxy.insertData("WordRecognized", [])
                self.sttVocabCounter = 0

            for wd, prob in zip(wds[0::2], wds[1::2]):
                print >>sys.__stdout__,"INFO", str(wd) +str(prob)
                if wd == word and prob > threshold:
                    self.proj.executor.postEvent("INFO", "Recognized word '%s' with p = %f" % (wd, prob))
                    return True

            return False


    def seePerson(self, initial=False):
        """
        Use Nao's face recognition to detect a person's face in the field of view.
        """

        if initial:
            ### Initialize face tracking

            if self.faceProxy is None:
                self.faceProxy = self.naoInitHandler.createProxy('ALFaceDetection')

                subs = [x[0] for x in self.faceProxy.getSubscribersInfo()]
                # Close any previous subscriptions that might have been hanging open
                if "ltlmop_sensorhandler" in subs:
                    self.faceProxy.unsubscribe("ltlmop_sensorhandler")
                self.faceProxy.subscribe("ltlmop_sensorhandler")

                return True
        else:
            # Check face detection state
            face_data = self.memProxy.getData("FaceDetected",0)
            return (face_data != [])

    def headTapped(self, initial=False):
        """
        Check whether the button on top of Nao's head is pressed.
        """

        if initial:
            if self.memProxy is None:
                self.memProxy = self.naoInitHandler.createProxy('ALMemory')
            return True
        else:
            if bool(self.memProxy.getData('FrontTactilTouched',0)):
                self.proj.executor.postEvent("INFO","sensor.py: headTapped is True now!")
            return bool(self.memProxy.getData('FrontTactilTouched',0))
    
    def headTappedBack(self, initial=False):
        """
        Check whether the back button on top of Nao's head is pressed.
        """

        if initial:
            if self.memProxy is None:
                self.memProxy = self.naoInitHandler.createProxy('ALMemory')
            return True
        else:
            if bool(self.memProxy.getData('RearTactilTouched',0)):
                self.proj.executor.postEvent("INFO","sensor.py: headTappedBack is True now!")    
            return bool(self.memProxy.getData('RearTactilTouched',0))

    def findChief(self,  initial=False):

        if initial:
            print "Connecting to Vicon server..."
            self.viconServer = _pyvicon.ViconStreamer()
            self.viconServer.connect("10.0.0.102", 800)
            
            model_name = "GPSReceiverHelmet-goodaxes:GPSReceiverHelmet01"
            self.viconServer.selectStreams(["Time"] + ["{} <{}>".format(model_name, s) for s in ("t-X", "t-Y")])
            self.viconServer.startStreams()
            
            # Wait for first data to come in
            while self.viconServer.getData() is None:
                pass
                
        else:
            (t, x, y) = self.viconServer.getData()
            (t, x, y) = [t/100, x/1000, y/1000]
            
            # Find our current configuration
            pose = [6410.0/1000, -768.0/1000]   # center of kitchen now
            
            range = 0.5
            
            if   math.sqrt((pose[0]-x)**2+(pose[1]-y)**2)<range:
                self.proj.executor.postEvent("INFO","See hat: currentPose: " + str(pose) + "currentHat: " + str(x) + str(y)  + "range: " + str(math.sqrt((pose[0]-x)**2+(pose[1]-y)**2))) 
                
            return math.sqrt((pose[0]-x)**2+(pose[1]-y)**2)<range
            
      
    def betweenClasses(self,  initial=False):

        if initial:
            print "Connecting to Vicon server..."
            self.viconServer2 = _pyvicon.ViconStreamer()
            self.viconServer2.connect("10.0.0.102", 800)
            
            model_name = "folder:mainBody" ##############
            self.viconServer2.selectStreams(["Time"] + ["{} <{}>".format(model_name, s) for s in ("t-X", "t-Y")])
            self.viconServer2.startStreams()
            
            # Wait for first data to come in
            while self.viconServer2.getData() is None:
                pass
                
        else:
            (t, x, y) = self.viconServer2.getData()
            (t, x, y) = [t/100, x/1000, y/1000]
            
            # Find our current configuration
            pose = [6331.0/1000, 584.0/1000]   # #########center of kitchen now
            
            range = 0.5
            if   math.sqrt((pose[0]-x)**2+(pose[1]-y)**2)<range:
                self.proj.executor.postEvent("INFO","See STH: currentPose: " + str(pose) + "currentHat: " + str(x) + str(y)  + "range: " + str(math.sqrt((pose[0]-x)**2+(pose[1]-y)**2))) 
                
            return math.sqrt((pose[0]-x)**2+(pose[1]-y)**2)<range
