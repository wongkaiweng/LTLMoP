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
two_robots = False

regions = [
[[ 483, 315, 312, 484],
 [ 627, 625, 502, 502]],
[[ 168, 163, 312, 315],
 [ 628, 500, 502, 625]],
[[  622, 483, 484, 623],
 [ 630, 627, 502, 502]],
[[ 618, 474, 474, 618],
 [ 229, 227, 92, 92]],
[[ 474, 304, 304, 474],
 [ 227, 227, 92, 92]],
[[484, 312, 304, 476, 484],
[502, 502, 370, 369, 501]],
[[304, 304, 474, 476],
[370, 227, 227, 369]],
[[159, 158, 304, 304],
[229, 91, 92, 227]]
]
coordmap_map2lab = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
currentDesIdx = 1

if two_robots:
    # two robots
    poseDic = OrderedDict([('rob2', [533, 563.25]), ('rob1', [231.25, 159.75])])
    futureDes = [[2 , 7], [2 , 4], [ 2, 6], [ 2, 5], [2 , 0], [2, 1], [0, 1], [5 , 1], [6 , 0]]
    next_regIndicesDict = OrderedDict([('rob2', futureDes[currentDesIdx][0]), ('rob1', futureDes[currentDesIdx][1])])
    arrived = {'rob2':False, 'rob1': False}

else:
    # three robots
    # poseDic = OrderedDict([('rob3', [513.8, 146.4]), ('rob2', [ 475.14121791 , 564.42375966]), ('rob1', [ 306.98208151 , 161.58181675])])
    # futureDes = [[3, 0, 4], [3, 1, 6]]
    poseDic = OrderedDict([('rob3', [513.8, 146.4]), ('rob2', [533, 563.25]), ('rob1', [231.25, 159.75])])
    # futureDes = [[3, 2, 7], [3, 2, 4], [3, 0, 4], [3, 0, 6], [3, 1, 6]]  # future destinations
    futureDes = [[3, 2, 7], [3, 0, 7], [3, 0, 4], [3, 0, 6], [3, 1, 6]]  # future destinations
    next_regIndicesDict = OrderedDict([('rob3', futureDes[currentDesIdx][0]), ('rob2', futureDes[currentDesIdx][1]), ('rob1', futureDes[currentDesIdx][2])])
    arrived = {'rob3':False, 'rob2':False, 'rob1': False}

session = MATLABPythonInterface.initializeMATLABPythonCommunication(regions, coordmap_map2lab)
count = 0
gain = 1
replan = [0]
while count < 20000:
    try:
        vx, vy, changes, currentLoc = MATLABPythonInterface.getMATLABVelocity(session, poseDic, next_regIndicesDict, replan)
    except:
        # try to get exception str from matlab
        session.putvalue('MSCRIPT', 'msgString = lasterror')
        session.run('eval(MSCRIPT)')
        session.putvalue('MSCRIPTmsg', 'errMsg = msgString.message')
        session.run('eval(MSCRIPTmsg)')
        session.putvalue('MSCRIPTIdentifier', 'errIdentifier = msgString.identifier')
        session.run('eval(MSCRIPTIdentifier)')
        logging.error(session.getvalue('errMsg'))
        logging.error(session.getvalue('errIdentifier'))

        # get size of stack
        session.putvalue('MSCRIPTstackSize', 'stackSize = size(msgString.stack)')
        session.run('eval(MSCRIPTstackSize)')
        stackSize = session.getvalue('stackSize')[0]
        logging.error('stackSize:' + str(session.getvalue('stackSize')) + ' stackSize:' + str(stackSize))
        for x in range(int(stackSize)):
            session.putvalue('MSCRIPTmsgStack', 'errStack' + str(x + 1) + '= msgString.stack(' + str(x + 1) + ',1)')
            session.run('eval(MSCRIPTmsgStack)')
            session.putvalue('MSCRIPTmsgStack' + str(x + 1) + 'File', 'errStack' + str(x + 1) + 'File = errStack' + str(x + 1) + '.file')
            session.run('eval(MSCRIPTmsgStack' + str(x + 1) + 'File)')
            # logging.error('file:' + str(self.session.getvalue('errStack' + str(x + 1) + 'File')))
            session.putvalue('MSCRIPTmsgStack' + str(x + 1) + 'Name', 'errStack' + str(x + 1) + 'Name = errStack' + str(x + 1) + '.name')
            session.run('eval(MSCRIPTmsgStack' + str(x + 1) + 'Name)')
            session.putvalue('MSCRIPTmsgStack' + str(x + 1) + 'Line', 'errStack' + str(x + 1) + 'Line = errStack' + str(x + 1) + '.line')
            session.run('eval(MSCRIPTmsgStack' + str(x + 1) + 'Line)')
            logging.error('file:' + str(session.getvalue('errStack' + str(x + 1) + 'File')) + \
                          'name:' + str(session.getvalue('errStack' + str(x + 1) + 'Name')) + \
                          'line:' + str(session.getvalue('errStack' + str(x + 1) + 'Line')))

    if vx.any():
        if two_robots:
            poseDic['rob2'][0] += vx[0] * gain
            poseDic['rob2'][1] += vy[0] * gain
            poseDic['rob1'][0] += vx[1] * gain
            poseDic['rob1'][1] += vy[1] * gain
        else:
            poseDic['rob3'][0] += vx[0] * gain
            poseDic['rob3'][1] += vy[0] * gain
            poseDic['rob2'][0] += vx[1] * gain
            poseDic['rob2'][1] += vy[1] * gain
            poseDic['rob1'][0] += vx[2] * gain
            poseDic['rob1'][1] += vy[2] * gain
    logging.debug(poseDic)
    logging.debug('---------------------------------------------')
    count += 1
    logging.debug('count = ' + str(count))
    if not changes.any():  # no changes in regions
        pass
        replan = [0]
        # print regions[next_regIndicesDict['rob1']], [poseDic['rob1'][0], poseDic['rob1'][1]]
        # arrived['rob1'] = is_inside([poseDic['rob1'][0], poseDic['rob1'][1]], np.array(regions[next_regIndicesDict['rob1']]))
        # arrived['rob2'] = is_inside([poseDic['rob2'][0], poseDic['rob2'][1]], np.array(regions[next_regIndicesDict['rob2']]))
        # if arrived['rob1'] and arrived['rob2']:
        #    currentDesIdx += 1
        #    next_regIndicesDict = OrderedDict([('rob2', futureDes[currentDesIdx][0]),('rob1',futureDes[currentDesIdx][1])])
    else:  # changes matrix is not empty
        print "region modified."
        currentDesIdx += 1
        replan = [1]
        if two_robots:
            next_regIndicesDict = OrderedDict([('rob2', futureDes[currentDesIdx][0]), ('rob1', futureDes[currentDesIdx][1])])

        else:
            next_regIndicesDict = OrderedDict([('rob3', futureDes[currentDesIdx][0]), ('rob2', futureDes[currentDesIdx][1]), ('rob1', futureDes[currentDesIdx][2])])
        time.sleep(5)



MATLABPythonInterface.closeInterface(session)
