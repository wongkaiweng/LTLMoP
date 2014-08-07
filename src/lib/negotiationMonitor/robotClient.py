#robotClient.py
import socket
import logging
import LTLParser.LTLcheck #for parsing region bits back to region names 
import ast   #for parsing spec dict from negtiation monitor
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
        self.regionList = []   #contains original names
        self.regions    = proj.rfi.regions # contains decomposed names
        
        #connect to the server
        self.clientObject = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.clientObject.connect((ADDR))
        
        #send out initial info
        self.initializeRegionExchange(hsub, proj)
        
        #find out mapping from new_names to old ones
        self.newRegionNameToOld = {}
        for rname, subregs in proj.regionMapping.iteritems():
            for newReg in subregs:
                self.newRegionNameToOld[newReg] = rname
    
    def initializeRegionExchange(self, hsub, proj):
        """
        This function obtains the name of the robot and also the list of region and send the info to the negotiation Monitor
        hsub:         self.hsub from LTLMoP
        proj:         self.proj from LTLMoP
        """
        self.robotName  = hsub.executing_config.getRobotByName(hsub.executing_config.main_robot).name
        self.regionList = [x.encode('ascii') for x, subregs in proj.regionMapping.iteritems()]
        
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
        self.clientObject.send(self.robotName + '-' + 'SysTrans = ' + sysSafety + '\n')
        logging.info('ROBOTCLIENT: send system safety from ' + str(self.robotName))

    def sendSysGoals(self, sysGoals):
        """
        This function sends the robot's system goals to the negotiation monitor. 
        """
        self.clientObject.send(self.robotName + '-' + 'SysGoals = ' + sysGoals + '\n')
        logging.info('ROBOTCLIENT: send system goals from ' + str(self.robotName))
        
    def requestEnvSafetyAssumptions(self):
        """
        This function requests the system guarantees of the other robots from the negotiation monitor.
        """
        self.clientObject.send(self.robotName + '-' + 'EnvTrans = ' + "''" '\n')
        logging.info('ROBOTCLIENT: request env safety of other robots')
        
        #receive info
        EnvTrans = ast.literal_eval(self.clientObject.recv(self.BUFSIZE))
        logging.info(EnvTrans)
        
        #LTLParser.LTLcheck.parseBitToRegion(EnvTrans['Bob'], self.regions, self.newRegionNameToOld)
        
    def requestEnvLivenesses(self):
        """
        This function requests the system goals of the other robots from the negotiation monitor. 
        """
        self.clientObject.send(self.robotName + '-' + 'EnvGoals = ' + "''" '\n')
        logging.info('ROBOTCLIENT: request env goals of other robots')
        
        #receive info
        EnvGoals = ast.literal_eval(self.clientObject.recv(self.BUFSIZE))
        logging.info(EnvGoals)
        

def findRegionBits(ltlFormula):
    """
    This function finds regions in bits in ltl formula.
    INPUT:
    ltlFormula: ltl formula in normal format
    OUTPUT:
    regionBitsList: list of regionbits strings 
    """
    pattern = "\(((!?next\(s.bit[0-9]\)&?)|(!?s.bit[0-9]&?))+\)"
    regionBitsList = [x.group() for x in re.finditer(pattern,ltl)]
    
    return regionBitsList
    
def matchRegionNumber(regionBitStr, regionList, newRegionNameToOld):
    """
    This function takes in a region bit string and regionList and return the actual region string (with next)
    INPUT:
    regionBitStr: region string in bits
    regionList: list of regions (in new names)
    newRegionNameToOld: dict from new names to old names (made from regionMapping)
    OUTPUT:
    targetRegionOrig: the original name of the region 
    """
    
    # figure out if the regionBitStr contains "next"
    if 'next' in regionBitStr:
        nextTimeStep = True
    else:
        nextTimeStep = False
    
    # isolate each bit
    pattern_bit = "!?(next\()?s.bit[0-9]\)?"
    individualBitsList = [x.group() for x in re.finditer(pattern_bit, regionBitStr)]
    
    regionNo = 0
    # calculate region number
    for bit in individualBitsList:
        bit = bit.replace('next(','').replace(')'.'')
        
        if '!' not in bit:
            regionNo += 2**int(bit)
            
    # find region in new name
    targetRegionNew = regionList[regionNo]
    
    # find reigon in old name
    targetRegionOrig = newRegionNameToOld[targetRegionNew]
    
    # return region name string
    if nextTimeStep is True:
        return 'next(' + targetRegionOrig + ')'
    else:
        return targetRegionOrig
        
def replaceAllRegionBitsToOriginalName(ltlFormula, regionList, newRegionNameToOld, robotName = ''):
    """
    This function takes in an ltlFormula with region bits, regionList and newRegionNameToOld, and replace all names to the original ones
    INPUT:
    ltlFormula: ltlFormula in normal form
    regionList: list of regions (in new names)
    newRegionNameToOld: dict from new names to old names (made from regionMapping)
    robotName: name of the robot (optional)
    OUTPUT:
    ltlFormulaReplaced: ltl formula with all regionBits replaced 
    """
    
    # make a copy of the string
    ltlFormulaReplaced = ltlFormula
    
    # find the list of bit regions
    regionBitsList = findRegionBits(ltlFormula)
    
    for regionBitStr in regionBitsList:
        # find original region name 
        regionName = matchRegionNumber(regionBitStr, regionList, newRegionNameToOld)
        
        # replace region bits to name
        if robotName is '':
            ltlFormulaReplaced = ltlFormulaReplaced.replace(regionBitStr,regionName)
        else:
            ltlFormulaReplaced = ltlFormulaReplaced.replace(regionBitStr,robotName + '_' + regionName)

    return ltlFormulaReplaced   
            
    
    
    
    
