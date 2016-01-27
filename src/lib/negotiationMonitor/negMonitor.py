#negMonitor.py
import socket #import the socket library
import logging
import select  #for multiple robot client connections
import re    #for parsing msg from client
import ast   #for parsing msg from client
import sys   #for program exiting

logging.basicConfig(level=logging.DEBUG)
#logger = logging.getLogger(__name__)

##let's set up some constants
HOST = ''    #we are the host
ADDR = ("localhost",6501)    #we need a tuple for the address
BUFSIZE = 2000000    #reasonably sized buffer for data
 
## now we create a new socket object (serv)
serv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)    

listenConn = 5  #listenConn is the maximum number of queued connections we'll allow

#find our socket to the address
serv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) # no one minute timeout for reconnection
serv.bind((ADDR))    #the double parens are to create a tuple with one element
serv.listen(listenConn) 
logging.info( 'NEGOTIATION_MONITOR: Listening...')

keepConnection = True

clients = [serv]
###############################################
############# region monitor ##################
###############################################        

regionList = {}  #tracking region info for each robot
spec       = {'SysTrans':{},'SysGoals':{},'EnvTrans':{},'EnvGoals':{}}
strategyStatus = {} #tracking strategy status of each robot (true for realizable. False otherwise.)
requestSpecStatus = {} # track what spec is being requested
negotiationStatus = None # track the negotiation status
negotiationInitiator = None # track who initiated negotiation
violationTimeStamp = {} # track the time stamp that safety violation is detected
robotSensors = {} # track list of sensors, esp after negotiation

def printRegionInfo():
    """
    This function prints the current region information of each robot
    """
    logging.info('----------------------------------------------')
    for region, robots in regionList.iteritems():
        table = ""
        for rob, status in robots.iteritems():
            table = table + "-{0:10}: {1:6} ".format(rob, status)    
        table = "{0:13}".format(region) + table
        logging.info(table)

def printSpec(specType, specStr, robotName):
    """
    This function prints the spec type and the spec.
    specType: specification type
    specStr : specification string
    robotName: name of the robot that has this specification
    """
    logging.info('===============================================')
    logging.info('==== ' + specType + ' of ' + robotName + ' ====')
    logging.info('===============================================')
    logging.info(specStr)
    logging.info('===============================================')
           

while keepConnection:
    try:
        inputReady, outputReady, exceptReady = select.select(clients, [], [])
        for x in inputReady: 

            if x == serv:  
                csock, addr = serv.accept() 
                clients.append(csock) 
                #logging.info('NEGOTIATION_MONITOR: ...connected! client list = ' + str(clients))

            else:  
                # find data pattern
                data = x.recv(BUFSIZE) 
                pattern ="(?P<robotName>\w+)-(?P<packageType>\w+)\s=\s(?P<packageValue>\[?.+\]?)\n"      # MSG FORMAT  
                result = re.finditer(pattern, data)

                for item in result:
                    if item.group('packageType')  == "regionList":
                        # save region info and parse list into a dict of the form  regionList[r1][robot] = true

                        # convert region list string to list
                        rList = ast.literal_eval(item.group("packageValue"))
                        
                        # store current status of the robot be all false
                        for reg in rList:
                            if not reg in regionList.keys():
                                regionList[reg] = {}
                            regionList[reg][item.group("robotName")] = False

                        printRegionInfo()
                        
                        # set up spec for the robot
                        for specType, value in spec.iteritems():
                            spec[specType][item.group("robotName")] = ""
                        
                        # initialize requestSpecStatus
                        requestSpecStatus[item.group("robotName")] = []
                        
                        # initialize strategy status
                        strategyStatus[item.group("robotName")] = None
                        
                        # initialize violation time stamp
                        violationTimeStamp[item.group("robotName")] = 0

                        # initialize robot sensor dict
                        robotSensors[item.group("robotName")] = {}
                        
                    elif item.group('packageType')  ==  "regionName":
                        # first figure out if it's rc region or not
                        RCregion = False
                        if item.group("packageValue").rfind('_rc') > 0:
                            RCregion = True

                        # update region info of robot
                        for region, robots in regionList.iteritems():
                            if regionList[region][item.group("robotName")] and ((region.rfind('_rc')>0) == RCregion):
                                regionList[region][item.group("robotName")] = False
                            
                            if item.group("packageValue") == region:
                                regionList[region][item.group("robotName")] =True
                                
                        printRegionInfo()

                    elif item.group('packageType') in ['SysTrans','SysGoals','EnvTrans','EnvGoals']:
                        if ast.literal_eval(item.group("packageValue")):
                            # We got spec from robotClient, save spec
                            spec[item.group('packageType')][item.group("robotName")] = ast.literal_eval(item.group("packageValue"))
                            printSpec(item.group('packageType'), spec[item.group('packageType')][item.group("robotName")], item.group("robotName"))
                            
                        else:
                            # robotClient is requesting spec, send back spec  
                            logging.info(item.group('packageType') + ' requested by ' + item.group("robotName"))              
                            # check if we need to request spec from the other robot
                            for robot, specStr in spec[item.group('packageType')].iteritems():
                                if robot != item.group("robotName") and spec[item.group('packageType')][robot] == "":
                                    requestSpecStatus[robot].append(item.group('packageType'))
                            
                            # send spec back to the robot
                            x.send(str(spec[item.group('packageType')]))
                            
                    elif item.group('packageType') == 'robotSensors':
                        if ast.literal_eval(item.group("packageValue")) != '':
                            robotSensors[item.group("robotName")] = ast.literal_eval(item.group("packageValue"))
                        else:
                            x.send(str(robotSensors))

                    elif item.group('packageType')  == "sensorUpdate":
                        # send the list of region info
                        x.send(str(regionList))
                    
                    elif item.group('packageType')  == "updateStrategyStatus":
                        # received controller info
                        strategyStatus[item.group("robotName")] = ast.literal_eval(item.group("packageValue"))
                        logging.info('NEGOTIATION_MONITOR: strategy status of ' + item.group("robotName") + ' received.')
                    
                    elif item.group('packageType')  == "requestStrategyStatus": 
                        # send controller info
                        x.send(str(strategyStatus))
                        logging.info('NEGOTIATION_MONITOR: strategy status sent to ' + item.group("robotName"))          
                    
                    elif item.group('packageType')  == "requestSpecStatus":
                        # send spec status back to the robot
                        x.send(str(requestSpecStatus[item.group('robotName')]))
                        # clear the list
                        requestSpecStatus[item.group('robotName')] = []
                    
                    elif item.group('packageType') == "negotiationStatus":
                        if ast.literal_eval(item.group("packageValue")) or ast.literal_eval(item.group("packageValue")) == False:
                            # We got set negotiationStatus from robotClient
                            negotiationStatus = item.group("packageValue")
                            negotiationInitiator = "'"+item.group('robotName')+"'"
                        else:
                            # send negotiationStatus back to the robot
                            x.send(";" + str(negotiationStatus)+'-'+str(negotiationInitiator))
                    
                    elif item.group('packageType') == "violationTimeStamp":
                        if ast.literal_eval(item.group("packageValue")) != '':
                            # We got set violationTimeStamp from robotClient
                            violationTimeStamp[item.group("robotName")] = ast.literal_eval(item.group("packageValue"))
                            logging.info('@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@')
                            logging.info('violationTimeStamp:'+ str(violationTimeStamp))
                            logging.info('@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@')
                        else:
                            # send violationTimeStamp back to the robot
                            x.send(str(violationTimeStamp))

                    elif "closeConnection" in data:
                        x.close() 
                        clients.remove(x)
                        logging.info('NEGOTIATION_MONITOR: client ' + str(x) + 'is removed.' )
                    
                    else:
                        pass
                
    except KeyboardInterrupt:
        for x in clients:
            x.close()
        serv.close()
        logging.info("NEGOTIATION_MONITOR: Closing Connection.") 
        sys.exit()        
        
