#robotClient.py
import socket               # for communication with negotiation monitor
import logging
import LTLParser.LTLRegion  # from replace region names in ltl
import ast                  # for parsing spec dict from negtiation monitor
import strategy             # for finding regions from region bits
import numpy                # for generating bit encoding
import parseEnglishToLTL    # for parsing original region name to region bits
import re                   # for parsing regionCompleted_b and region_b to sbit and bit

#logging.basicConfig(level=logging.DEBUG)
#logger = logging.getLogger(__name__)

class RobotClient:

    """
    LTLMoP robot object used to communicate with the negotiationMonitor
    """
    def __init__(self, hsub, proj):
        ADDR = ("localhost",6501)
        self.BUFSIZE = 20000

        # check if we are using fastslow:
        self.fastslow = proj.compile_options['fastslow']

        # initialize our variable
        self.robotName = ''
        self.regions    = proj.rfi.regions # contains decomposed names
        if self.fastslow:
            self.regionCompleted_domain = strategy.Domain("regionCompleted",  proj.rfi.regions, strategy.Domain.B0_IS_MSB)
        self.region_domain = strategy.Domain("region",  proj.rfi.regions, strategy.Domain.B0_IS_MSB)

        #find out mapping from new_names to old ones
        self.newRegionNameToOld = {}
        for rname, subregs in proj.regionMapping.iteritems():
            for newReg in subregs:
                self.newRegionNameToOld[newReg] = rname
                
        # form regionList with original names
        self.regionList = []
        for region in self.regions:
            self.regionList.append(self.newRegionNameToOld[region.name])
        
        # for mapping original region name to bit encoding
        self.bitEncode = parseEnglishToLTL.bitEncoding(len(self.regionList), int(numpy.ceil(numpy.log2(len(self.regionList)))))       
        
        #connect to the server
        self.clientObject = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.clientObject.connect((ADDR))
        
        #send out initial info
        self.initializeRegionExchange(hsub)
        #self.initializeCompletedRegionExchange()
         
        # track if spec is requested
        self.specRequestFromOther = [] # list of spec requested

    def initializeRegionExchange(self, hsub):
        """
        This function obtains the name of the robot and also the list of region and send the info to the negotiation Monitor
        hsub:         self.hsub from LTLMoP
        """
        self.robotName  = hsub.executing_config.getRobotByName(hsub.executing_config.main_robot).name
        
        # send region info to negotiation monitor
        self.clientObject.send(self.robotName +'-' + 'regionList = ' + str(self.regionList) + '\n')
        logging.info("ROBOTCLIENT: initialize region info from " + str(self.robotName))
        
    def updateRobotRegion(self, current_region):
        """
        This function update the region info in the negotiation monitor if the robot is at a next region
        current_region: region object in LTLMoP
        """
        # send current region to negotiation monitor       
        self.clientObject.send(self.robotName + '-' + 'regionName = ' + str(self.newRegionNameToOld[current_region.name]) + '\n')
        logging.info("ROBOTCLIENT: update region info from " + str(self.robotName))

    def initializeCompletedRegionExchange(self):
        """
        This function sends the list of completed region to the negotiation Monitor
        hsub:         self.hsub from LTLMoP
        """
        # send region info to negotiation monitor
        self.clientObject.send(self.robotName +'-' + 'regionList = ' + str([reg+'_rc' for reg in self.regionList]) + '\n')
        logging.info("ROBOTCLIENT: initialize region info from " + str(self.robotName))

    def updateCompletedRobotRegion(self, current_region):
        """
        This function update the completed region info in the negotiation monitor if the robot is at a next region
        current_region: region object in LTLMoP
        """
        # send current region to negotiation monitor
        self.clientObject.send(self.robotName + '-' + 'regionName = ' + str(self.newRegionNameToOld[current_region.name]+'_rc') + '\n')
        logging.info("ROBOTCLIENT: update region info from " + str(self.robotName))

    def closeConnection(self):
        """
        This function closes the connection with the negotiation monitor.
        """    
        self.clientObject.close()
        logging.info('ROBOTCLIENT: connection to the negotiation monitor is now closed')
   
    def sendSpec(self, specType, spec):
        """
        This function sends the robot's spec to the negotiation monitor according to the specType.
        """
        # check if the specType is valid
        possibleSpecTypes = ['SysInit','SysTrans','SysGoals','EnvInit','EnvTrans','EnvGoals']
        if specType not in possibleSpecTypes:
            raise TypeError('specType must be ' + str(possibleSpecTypes))
        
        #spec = spec.replace('\t',"").replace(' ','').replace('\n','')
        spec = spec.replace(' ','').replace('\n','')

        # first replace our region bits to original region name with our robot name
        if self.fastslow:
            spec =  LTLParser.LTLRegion.replaceAllRegionBitsToOriginalName(spec, self.regions, self.regionCompleted_domain, self.newRegionNameToOld, self.robotName, self.fastslow)

        spec =  LTLParser.LTLRegion.replaceAllRegionBitsToOriginalName(spec, self.regions, self.region_domain, self.newRegionNameToOld, self.robotName, False)

        # send sysSafety to negotiation monitor
        self.clientObject.send(self.robotName + "-" + specType + " = '" + spec + "'\n")
        logging.info('ROBOTCLIENT: send '+ specType +' from ' + str(self.robotName))
        
    
    def requestSpec(self, specType):
        """
        This function requests the system guarantees of the other robots from the negotiation monitor.
        OUTPUT:
        specToAppend: ltl formula ready to append to envTrans
        """
        # check if the specType is valid
        possibleSpecTypes = ['SysTrans','SysGoals','EnvTrans','EnvGoals']
        if specType not in possibleSpecTypes:
            raise TypeError('specType must be ' + str(possibleSpecTypes))
            
        specToAppend = ""
        
        #TODO: need to get full picture of all robots. The current solution only deals with two robots
        while not len(specToAppend):
            self.clientObject.send(self.robotName + '-' + specType +' = ' + "''" '\n')
            logging.info('ROBOTCLIENT: request '+ specType + ' of other robots')

            #receive info
            SpecDict = ast.literal_eval(self.clientObject.recv(self.BUFSIZE))

            for robot, spec in SpecDict.iteritems():
                #self.robotName = 'alice' #TODO: remove this later
                if self.robotName  != robot:
            
                    # change region props with our name to region bits (parseEnglishToLTL?)              
                    specToAppend += LTLParser.LTLRegion.replaceRobotNameWithRegionToBits(spec, self.bitEncode, self.robotName, self.regionList, self.fastslow)

        return specToAppend 
        
    def requestRegionInfo(self):
        """
        This function request the region dict of all robots.
        OUTPUT: 
        robotRegionStatus: dict containing all region info
        """
        self.clientObject.send(self.robotName +'-' + 'sensorUpdate = ' + "''" + '\n')
        robotRegionStatus = ast.literal_eval(self.clientObject.recv(self.BUFSIZE))
        
        return robotRegionStatus
        
    def updateStrategyStatus(self,realizable):
        """
        This function update the current controller status to the negotiation monitor.
        INPUT: 
        realizable: True or False
        """
        self.clientObject.send(self.robotName +'-' + 'updateStrategyStatus = ' + str(realizable) + '\n')
        
    def requestStrategyStatus(self):
        """
        This function requests the controller status of the other robots from the negotiation monitor.
        OUTPUT:
        realizable: dict of boolean. realizable['rob1'] = True
        """
        self.clientObject.send(self.robotName + '-' + 'requestStrategyStatus = ' + "''" + '\n')
        logging.info('ROBOTCLIENT: request strategy status of other robots')

        #receive info
        realizable = ast.literal_eval(self.clientObject.recv(self.BUFSIZE))
        logging.debug(realizable)
        
        return realizable
    
    def checkRequestSpec(self):
        """
        This function check if our specification is currently requested by the other robot
        """
        self.clientObject.send(self.robotName + '-' + 'requestSpecStatus = ' + "''" +  '\n')
        #logging.info('ROBOTCLIENT: check request spec status of other robots')
        
        #receive info
        self.specRequestFromOther = ast.literal_eval(self.clientObject.recv(self.BUFSIZE))
        #logging.debug(self.specRequestFromOther)
   
    def checkNegotiationStatus(self):
        """
        This function check the current negotiation status.
        None     : no negotiation has started.
        robotName: This robot should resynthesize the specification with the spec snippets from the other robot.
        True     : negotiation is completed.
        False    : negotiation failed. Alternatives seeked.
        """
        self.clientObject.send(self.robotName + '-' + 'negotiationStatus = ' + "''" + '\n')
        
        #receive info
        originalStr = self.clientObject.recv(self.BUFSIZE)
        bufferData = originalStr.split(':').pop()
        negotiationStatus = ast.literal_eval(bufferData)
        return negotiationStatus
        
    def setNegotiationStatus(self, status):
        """
        This function set the current negotiation status.
        None     : no negotiation has started.
        robotName: This robot should resynthesize the specification with the spec snippets from the other robot.
        True     : negotiation is completed.
        False    : negotiation failed. Alternatives seeked.
        """

        self.clientObject.send(self.robotName + '-' + 'negotiationStatus = ' + str(status) + '\n')
        logging.info('ROBOTCLIENT: negotiation status set')
        
    def getViolationTimeStamp(self, otherRobotName):
        """
        This function gets the violation time stamp.
        """
        self.clientObject.send(self.robotName + '-' + 'violationTimeStamp = ' + "''" + '\n')
        timeStamp = ast.literal_eval(self.clientObject.recv(self.BUFSIZE))

        return timeStamp[otherRobotName]
        
    def setViolationTimeStamp(self, timeStamp):
        """
        This function sets the violation time stamp.
        """
        self.clientObject.send(self.robotName + '-' + 'violationTimeStamp = ' + str(timeStamp) + '\n')

    def sendProp(self, propListType, propDict):
        """
        This function sends environment propsitions with values
        propListType: either 'sys' or 'env'
        propDict    : {propName:propValue}
        """
        # send prop
        def decorate_prop(prop):
            """
            replace regionCompleted_b and region_b to sbit and bit
            """
            prop = re.sub(r'region_b(\d+)$', r'bit\1', prop)
            prop = re.sub(r'regionCompleted_b(\d+)$', r'sbit\1', prop)
            return prop

        #make sure the region names are converted
        if propListType == 'env':
            propDictOnlyReg = {prop:value for prop, value in propDict.iteritems() if 'regionCompleted_b' in prop}
            propDict = {prop:value for prop, value in propDict.iteritems() if not 'regionCompleted_b' in prop}

            # find region in new name
            targetRegionNew = self.regions[self.regionCompleted_domain.propAssignmentsToNumericValue(propDictOnlyReg)]

            # find reigon in old name
            targetRegionOrig = self.newRegionNameToOld[targetRegionNew.name]

            # append regions to dictionary
            for origReg in self.regionList:
                if origReg == targetRegionOrig:
                    propDict.update({self.robotName+'_'+origReg+'_rc':True})
                else:
                    propDict.update({self.robotName+'_'+origReg+'_rc':False})

            self.clientObject.send(self.robotName + '-' + 'envPropList = ' + str(propDict) + '\n')

        else:
            propDictOnlyReg = {prop:value for prop, value in propDict.iteritems() if 'region_b' in prop}
            propDict = {prop:value for prop, value in propDict.iteritems() if not 'region_b' in prop}

            # find region in new name
            targetRegionNew = self.regions[self.region_domain.propAssignmentsToNumericValue(propDictOnlyReg)]

            # find reigon in old name
            targetRegionOrig = self.newRegionNameToOld[targetRegionNew.name]

            # append regions to dictionary
            for origReg in self.regionList:
                if origReg == targetRegionOrig:
                    propDict.update({self.robotName+'_'+origReg:True})
                else:
                    propDict.update({self.robotName+'_'+origReg:False})

            self.clientObject.send(self.robotName + '-' + 'sysPropList = ' + str(propDict) + '\n')
        logging.info('ROBOTCLIENT: sent'+propListType+'propositions list with value')

    def setCoordinationStatus(self, patchingStatus):
        """
        This function sets the patching status.
        """
        self.clientObject.send(self.robotName + '-' + 'patchingStatus = ' + str(patchingStatus) +  '\n')
        logging.info('ROBOTCLIENT: set coorindation status to ' + str(patchingStatus))

    def checkCoordinationStatus(self):
        """
        This function checks if patching is initiated.
        """
        self.clientObject.send(self.robotName + '-' + 'patchingStatus = ' + "''" +  '\n')
        status = ast.literal_eval(self.clientObject.recv(self.BUFSIZE))

        return status
