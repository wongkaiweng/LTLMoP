import __MATLABPythonInterface as MATLABPythonInterface
import logging
from collections import OrderedDict
from __is_inside import *
import numpy as np
logging.basicConfig(level=logging.DEBUG)
import time
"""
next_regIndicesDict = OrderedDict([('rob1', 2),('rob2',3),('rob3',3)])
regions = [
[[ -2, -2, 0, 0],
 [ 0, 0, 0, 2]],
[[ 0, 0,2,2],
 [ 2,0,0,2]],
[[  2,2,4,4],
 [ 2,0,0,2]],
[[ 2,2,4,4],
 [ 4,2,2,4]],
[[ 2,2,4,4],
 [ 0,-2,-2,0]]
]
coordmap_map2lab = [[1,0,0],[0,1,0],[0,0,1]]
poseDic  = OrderedDict([('rob1',[-1 ,.5]),('rob2',[1,1]),('rob3',[3.5 , -1])])
"""

regions = [
[[ 483, 315, 312, 484],
 [ 627, 625, 502, 502]],
[[ 168, 163,312,315],
 [ 628,500,502,625]],
[[  622,483,484,623],
 [ 630,627,502,502]],
[[ 618,474,474,618],
 [ 229,227,92,92]],
[[ 474,304,304,474],
 [ 227,227,92,92]],
[[484,312,304,476,484],
[502,502,370,369,501]],
[[304,304,474,476],
[370,227,227,369]],
[[159,158,304,304],
[229,91,92,227]]
]
coordmap_map2lab = [[1,0,0],[0,1,0],[0,0,1]]

# test
poseDic  = OrderedDict([('rob2',[533,563.25]),('rob1',[231.25,159.75])])
futureDes = [[2 ,7],[2 ,4], [ 2, 6], [ 2, 5], [2 ,0], [2, 1], [0, 1], [5 ,1], [6 ,0]]
#poseDic  = OrderedDict([('rob2',[533,563.25]),('rob1',[312.91,540.18])])
#futureDes = [[2, 1], [0, 1], [5 ,1], [6 ,0]]

currentDesIdx = 1
next_regIndicesDict = OrderedDict([('rob2', futureDes[currentDesIdx][0]),('rob1',futureDes[currentDesIdx][1])])

arrived = {'rob1':False, 'rob2':False}
# error1
#poseDic  = OrderedDict([('rob2',[509,604]),('rob1',[453, 502])])
#next_regIndicesDict = OrderedDict([('rob2', 2),('rob1',1)])

session = MATLABPythonInterface.initializeMATLABPythonCommunication(regions, coordmap_map2lab)
count = 0
gain  = 1
while count < 20000:
    vx, vy, changes = MATLABPythonInterface.getMATLABVelocity(session, poseDic, next_regIndicesDict)

    if vx.any():
        poseDic['rob2'][0] += vx[0]*gain
        poseDic['rob2'][1] += vy[0]*gain
        poseDic['rob1'][0] += vx[1]*gain
        poseDic['rob1'][1] += vy[1]*gain
    logging.debug(poseDic)
    logging.debug('---------------------------------------------')
    count += 1
    logging.debug('count = ' + str(count))
    if not changes.any(): # no changes in regions
        pass
        #print regions[next_regIndicesDict['rob1']], [poseDic['rob1'][0], poseDic['rob1'][1]]
        #arrived['rob1'] = is_inside([poseDic['rob1'][0], poseDic['rob1'][1]], np.array(regions[next_regIndicesDict['rob1']]))
        #arrived['rob2'] = is_inside([poseDic['rob2'][0], poseDic['rob2'][1]], np.array(regions[next_regIndicesDict['rob2']]))
        #if arrived['rob1'] and arrived['rob2']:
        #    currentDesIdx += 1
        #    next_regIndicesDict = OrderedDict([('rob2', futureDes[currentDesIdx][0]),('rob1',futureDes[currentDesIdx][1])])
    else: # changes matrix is not empty
        print "region modified."
        currentDesIdx += 1
        next_regIndicesDict = OrderedDict([('rob2', futureDes[currentDesIdx][0]),('rob1',futureDes[currentDesIdx][1])])
        time.sleep(5)
        
    
    
MATLABPythonInterface.closeInterface(session)