#robotClient.py
import socket
import logging
#logging.basicConfig(level=logging.DEBUG)
#logger = logging.getLogger(__name__)

"""
#################################################
######### Start Exchanging Information ##########
#################################################

connection = True
while connection:

    ###############################################
    ###### send the regionMonitor region info #####
    ###############################################
    #--------------------  #
    #----- initialize ---  #
    #--------------------  #
    
    initializeRegionExchange(cli, self.hsub, self.proj)
    
    
    #---------------------- #
    #---- actualy monitor-- #
    #---------------------- #
    if self.current_region != self.next_region:
        
    
    # not sending info until region is changed
    cli.send('hello from client')
    data = cli.recv(BUFSIZE)
    if data == "connection closed":
        connection = False
    elif data != '':
        logging.info(data)
    else:
        continue
    
    ###############################################
    ######## for safety exchange later ############
    ###############################################
    
cli.close()
"""
class RobotClient:

    """
    LTLMoP robot object used to communicate with the negotiationMonitor
    """
    def __init__(self, hsub, proj):
        ADDR = ("localhost",6501)
        BUFSIZE = 4096
        
        # initialize our variable
        self.robotName = ''
        self.regionList = []
        
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
        
        # send info
        self.clientObject.send(self.robotName +'-' + 'regionList = ' + str(self.regionList) + '\n') # region domain list 
        logging.info("ROBOTCLIENT: initialize region info from " + str(self.robotName))
        
    def updateRobotRegion(self, current_region):
        """
        This function update the region info in the negotiation monitor if the robot is at a next region
        current_region: region object in LTLMoP
        """
        # send info       
        self.clientObject.send(self.robotName + '-' + 'regionName = ' + str(self.newRegionNameToOld[current_region.name]) + '\n') # region domain list 
        logging.info("ROBOTCLIENT: update region info from " + str(self.robotName))
        
    def closeConnection(self):
        """
        This function closes the connection with the negotiation monitor.
        """    
        self.clientObject.close()
        logging.info('"ROBOTCLIENT: connection to the negotiation monitor is now closed')
    
    
    
