#robotClient.py
import socket               # for communication with negotiation monitor
import logging
import LTLParser.LTLRegion  # from replace region names in ltl
import ast                  # for parsing spec dict from negtiation monitor
import strategy             # for finding regions from region bits
import numpy                # for generating bit encoding
import parseEnglishToLTL    # for parsing original region name to region bits

#logging.basicConfig(level=logging.DEBUG)
#logger = logging.getLogger(__name__)

class RobotClient:

    """
    LTLMoP robot object used to communicate with the negotiationMonitor
    """
    def __init__(self, hsub, proj):
        ADDR = ("localhost",6501)
        self.BUFSIZE = 4096
        
        # initialize our variable
        self.robotName = ''
        self.regions    = proj.rfi.regions # contains decomposed names
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
        
    def closeConnection(self):
        """
        This function closes the connection with the negotiation monitor.
        """    
        self.clientObject.close()
        logging.info('ROBOTCLIENT: connection to the negotiation monitor is now closed')
    
    def sendSysSafetyGuarantees(self, sysSafety):
        """
        This function sends the robot's system safety guarantees to the negotiation monitor.
        """
        sysSafety = sysSafety.replace('\t',"").replace(' ','').replace('\n','') 
        
        # first replace our region bits to original region name with our robot name
        sysSafety =  LTLParser.LTLRegion.replaceAllRegionBitsToOriginalName(sysSafety, self.regions, self.region_domain, self.newRegionNameToOld, self.robotName)
        
        # send sysSafety to negotiation monitor
        self.clientObject.send(self.robotName + '-' + 'SysTrans = ' + sysSafety + '\n')
        logging.info('ROBOTCLIENT: send system safety from ' + str(self.robotName))
        logging.debug('sysSafety:' + sysSafety)

    def sendSysGoals(self, sysGoals):
        """
        This function sends the robot's system goals to the negotiation monitor. 
        """
        sysGoals = sysGoals.replace('\t',"").replace(' ','').replace('\n','')
        
        # first replace our region bits to original region name with our robot name
        sysGoals =  LTLParser.LTLRegion.replaceAllRegionBitsToOriginalName(sysGoals, self.regions, self.region_domain, self.newRegionNameToOld, self.robotName)
        
        # send sysGoals to negotiation monitor
        self.clientObject.send(self.robotName + '-' + 'SysGoals = ' + sysGoals + '\n')
        logging.info('ROBOTCLIENT: send system goals from ' + str(self.robotName))
        logging.debug('sysGoals:' + sysGoals)
        
    def requestEnvSafetyAssumptions(self):
        """
        This function requests the system guarantees of the other robots from the negotiation monitor.
        OUTPUT:
        specToAppend: ltl formula ready to append to envTrans
        """
        self.clientObject.send(self.robotName + '-' + 'EnvTrans = ' + "''" '\n')
        logging.info('ROBOTCLIENT: request env safety of other robots')
        
        #receive info
        EnvTrans = ast.literal_eval(self.clientObject.recv(self.BUFSIZE))
        logging.debug(EnvTrans)
        
        # change names to fit our own spec
        specToAppend = ""
        for robot, spec in EnvTrans.iteritems():
            #self.robotName = 'alice' #TODO: remove this later
            if self.robotName  != robot:
        
                # change region props with our name to region bits (parseEnglishToLTL?)              
                specToAppend += LTLParser.LTLRegion.replaceRobotNameWithRegionToBits(spec, self.bitEncode, self.robotName, self.regionList)
                
        logging.debug(specToAppend)
        return specToAppend 
        
        
    def requestEnvLivenesses(self):
        """
        This function requests the system goals of the other robots from the negotiation monitor. 
        OUTPUT:
        specToAppend: ltl formula ready to append to envGoals
        """
        self.clientObject.send(self.robotName + '-' + 'EnvGoals = ' + "''" '\n')
        logging.info('ROBOTCLIENT: request env goals of other robots')
        
        #receive info
        EnvGoals = ast.literal_eval(self.clientObject.recv(self.BUFSIZE))
        logging.debug(EnvGoals)
        
        # change names to fit our own spec
        specToAppend = ""
        for robot, spec in EnvGoals.iteritems():
            if self.robotName  != robot:
                
                # change region props with our name to region bits (parseEnglishToLTL?)
                specToAppend += LTLParser.LTLRegion.replaceRobotNameWithRegionToBits(spec, self.bitEncode, self.robotName, self.regionList)
           
        logging.debug(specToAppend)
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

    
