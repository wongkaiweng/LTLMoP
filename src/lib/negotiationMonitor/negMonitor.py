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
BUFSIZE = 4096    #reasonably sized buffer for data
 
## now we create a new socket object (serv)
serv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)    

listenConn = 5  #listenConn is the maximum number of queued connections we'll allow

#find our socket to the address
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
                logging.info('NEGOTIATION_MONITOR: ...connected! client list = ' + str(clients))

            else:  
                # find data pattern
                data = x.recv(BUFSIZE) 
                pattern ="(?P<robotName>\w+)-(?P<packageType>\w+)\s=\s(?P<packageValue>\[?.+\]?)\n"      # MSG FORMAT  
                result = re.finditer(pattern, data)
                
                for item in result:
                    if item.group('packageType')  == "regionList":
                        # save region info and parse list into a dict of the form  regionList[robot][r1] = true
                        # TODO:need to make sure two robots have the same regions

                        # convert region list string to list
                        rList = ast.literal_eval(item.group("packageValue"))
                        
                        # store current status of the robot be all false
                        if not len(regionList): #regionlist is empty. add all regions 
                            for reg in rList:
                                regionList[reg] = {}
                                regionList[reg][item.group("robotName")] = False
                        else: #regionList already existed
                            for reg in rList:
                                try:
                                    regionList[reg][item.group("robotName")] = False
                                except:
                                    logging.error('NEGOTIATION_MONITOR: The region list of robots does not match!')
                       
                        printRegionInfo()
                        
                    elif item.group('packageType')  ==  "regionName":
                        # update region info of robot
                        for region, robots in regionList.iteritems():
                            if regionList[region][item.group("robotName")] == True:
                                regionList[region][item.group("robotName")] = False
                            
                            if item.group("packageValue") == region:
                                regionList[region][item.group("robotName")] =True
                                
                        printRegionInfo()
                    
                    elif item.group('packageType') in ['SysTrans','SysGoals','EnvTrans','EnvGoals']:
                        if ast.literal_eval(item.group("packageValue")):
                            # We got spec from robotClient, save spec
                            spec[item.group('packageType')][item.group("robotName")] = item.group("packageValue")
                            printSpec(item.group('packageType'), spec[item.group('packageType')][item.group("robotName")], item.group("robotName"))
                            
                        else:
                            # robotClient is requesting spec, send back spec                  
                            x.send(str(spec[item.group('packageType')]))
                    
                    elif item.group('packageType')  == "sensorUpdate":
                        # send the list of region info
                        x.send(str(regionList))
                    
                    elif item.group('packageType')  == "updateStrategyStatus":
                        # received controller info
                        strategyStatus[item.group("robotName")] = item.group("packageValue")
                    
                    elif item.group('packageType')  == "requestStrategyStatus": 
                        # send controller info    
                        x.send(str(strategyStatus))               
                     
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
        
