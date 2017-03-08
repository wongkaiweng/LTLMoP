#!/usr/bin/env python
"""
====================================================
naoSensors.py - Sensor handler for the Aldebaran Nao
====================================================
"""
# logger for ltlmop
import logging
ltlmop_logger = logging.getLogger('ltlmop_logger')
import threading, socket
import time

import lib.handlers.handlerTemplates as handlerTemplates

class NaoSensorHandler(handlerTemplates.SensorHandler):
    def __init__(self, executor, shared_data):
        self.naoInitHandler = shared_data['NAO_INIT_HANDLER']

        self.sttProxy = None
        self.sttVocabulary = []
        self.sttVocabCounter = 0

        self.faceProxy = None
        self.memProxy = None
        self.sttProxy = None
        self.ldmProxy = None
        self.soundProxy = None
        self.sonarProxy = None
        self.motionProxy = None
        self.behaviorProxy = None
        if executor:
            self.proj = executor.proj
            self.executor = executor

        self.behaviorCompleted = {} # true if completed. False otherwise
        self.behaviorCompletionThread = {} # store threads for updating completion

        self.landmark_timestamp = {}

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

            self.landmark_timestamp[landMark_id] = None
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
                            self.landmark_timestamp[landMark_id] = time.time()
                            return True

                except Exception, e:
                    print "Naomarks detected, but it seems getData is invalid. ALValue ="
                    print val
                    print "Error msg %s" % (str(e))

            if self.landmark_timestamp[landMark_id] and \
                time.time() - self.landmark_timestamp[landMark_id] < 2:
                return True
            else:
                self.landmark_timestamp[landMark_id]  = None
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
                if wd == word and prob > threshold:
                    self.executor.postEvent("INFO", "Recognized word '%s' with p = %f" % (wd, prob))
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
                self.executor.postEvent("INFO","sensor.py: headTapped is True now!")
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
                self.executor.postEvent("INFO","sensor.py: headTappedBack is True now!")
            return bool(self.memProxy.getData('RearTactilTouched',0))

    def handIsClosed(self, hand=True, threshold=0.1, useSensors=False):
        """
        UNTESTED
        This function checks if one the the hands is closed.
        hand (bool): True for left hand, False for right hand. (default=True)
        threshold (float): Minimum angle that determines if the hand is closed. Hand aperture in percentage; 0 means closed, 1 means opened. (default=0.1,min=0.0,max=1.0)
        useSensors (bool) : True to use sensor angle values. False otherwise. (default=False)
        """
        if initial:
            if self.motionProxy is None:
               self.motionProxy = self.naoInitHandler.createProxy('ALMotion')
        else:
            get_angle_str = 'LHand' if hand else 'RHand'
            hand_angle = self.motionProxy.getAngles(get_angle_str, useSensors)[0]
            ltlmop_logger.debug(get_angle_str + '-hand_angle:' + str(hand_angle))
            if hand_angle < threshold: # adjust threshold
                # something in hand'
                ltlmop_logger.debug('hand closed')
                return True
            else:
                # nothing in hand
                return False

    def senseObstacle(self, threshold, initial=False):
        """
        UNTESTED
        Check whether the two chest sonars sense any obstacles.
        threshold (float): Minimum distance between the robot and the obstacle in meters(default=0.5,min=0.25,max=1.0)
        """
        # note that under 0.25m, the robot can sense the obstacle is present but cannot return a number
        if initial:
            if self.memProxy is None:
                self.memProxy = self.naoInitHandler.createProxy('ALMemory')
            if self.sonarProxy is None:
                self.sonarProxy = self.naoInitHandler.createProxy('ALSonar')
                self.sonarProxy.unsubscribe('LTLMoP') # first make sure we did unsubsribe
                self.sonarProxy.subscribe('LTLMoP')
            return True
        else:
            if self.memProxy.getData("Device/SubDeviceList/US/Left/Sensor/Value") < threshold or \
                self.memProxy.getData("Device/SubDeviceList/US/Right/Sensor/Value") < threshold:
                return True
                ltlmop_logger.debug('senseObstacle')
            else:
                return False

    def isBehaviorCompleted(self, behaviorName, initial=False):
        """
        Check if the behavior is currently running.
        behaviorName (string): name of the behavior.
        """
        def checkCompletion(behaviorName):
            running = False
            while True:
                if behaviorName in self.naoInitHandler.behaviorStatus.keys():
                    # return true when either actuator is true and completed. Stay true till actuator is false.
                    if self.naoInitHandler.behaviorStatus[behaviorName]:
                        if self.behaviorProxy.isBehaviorRunning(behaviorName):
                            # behavior started
                            running = True
                            #ltlmop_logger.debug('behavior started.')

                        elif running and not self.behaviorProxy.isBehaviorRunning(behaviorName):
                            # ran and then stopped. Action completed.
                            running = False
                            self.behaviorCompleted[behaviorName] = True

                            # broadcast msg that ingredient is delivered/received
                            def _broadcastBool(value):
                                """
                                send either True or False.
                                """
                                s=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                                s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
                                s.sendto(str(value),('255.255.255.255',12345))

                            if behaviorName == "passitem-763a4f/behavior_1":
                                ltlmop_logger.log(4, "ingredientReceived is not set to False.")
                                _broadcastBool(False)
                            #ltlmop_logger.debug('behavior completed.')

                        #ltlmop_logger.log(4,'behavior is true')
                        #ltlmop_logger.log(2,self.behaviorProxy.isBehaviorRunning(behaviorName))
                    else:
                        time.sleep(0.5)
                        self.behaviorCompleted[behaviorName] = False
                        #ltlmop_logger.debug('behavior completion ended.')
                else:
                    ltlmop_logger.warning("Behavior:" + behaviorName + " is not ready for status retrievel.")

        if initial:
            if self.behaviorProxy is None:
                self.behaviorProxy = self.naoInitHandler.createProxy('ALBehaviorManager')

            self.behaviorCompleted[behaviorName] = False #initialize for this behavior
            self.behaviorCompletionThread[behaviorName] = threading.Thread(target=checkCompletion, args=(behaviorName, ))
            self.behaviorCompletionThread[behaviorName].daemon = True  # Daemonize thread. exit when exception occurs
            self.behaviorCompletionThread[behaviorName].start()
        else:
            return self.behaviorCompleted[behaviorName]


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
                print >>sys.__stdout__,"See hat: currentPose: " + str(pose) + "currentHat: " + str(x) + str(y)  + "range: " + str(math.sqrt((pose[0]-x)**2+(pose[1]-y)**2)) 
                
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
                print >>sys.__stdout__,"See STH: currentPose: " + str(pose) + "currentHat: " + str(x) + str(y)  + "range: " + str(math.sqrt((pose[0]-x)**2+(pose[1]-y)**2))
                
            return math.sqrt((pose[0]-x)**2+(pose[1]-y)**2)<range
