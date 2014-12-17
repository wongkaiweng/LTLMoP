import pymatlab
import numpy as np
import logging
from collections import OrderedDict
import time  # for pause

two_robots = False
experimentInLab = False
threshold = 300

if two_robots:  # in pixels
    robRadius = OrderedDict([('rob2', 0.5), ('rob1', 0.5)])  # used to be 0.5
else:  # in pixels
    robRadius = OrderedDict([('rob3', 20), ('rob2', 20), ('rob1', 20)])  # used to be 0.5


# robRadius = OrderedDict([('rob1',0.5), ('rob2',0.5),('rob3',1)])
robots = robRadius

def initializeMATLABPythonCommunication(regions, coordmap_map2lab):
    """
    intialize communication with MATLAB. send robotRadius, threshold and regions.
    regions: self.rfi.regions
    coordmap_map2lab: self.coordmap_map2lab
    """
    logging.info('Starting MATLAB session...')
    session = pymatlab.session_factory()
    logging.info('MATLAB Session initialized ...')

    ####################################
    ###### run initialization ##########
    ####################################
    # function [vx vy changes currentloc]=getvelocity(pose,threshold,vertices, robots, destination)
    # Takes as input pose (n x d pose of all robots), threshold (how far robots
    # can coordinate), vertices (vertices of all regions as a cell array), currentrobot (the
    # index n_i of the current robot), and robots (an n x 1 array that contains
    # the size of each robot) destination (1xn array) contains the destination
    # cell for each robot

    #-------------------------------------------------------------------
    #------PYTHON: robotThreshold, MATLAB: threshold -------------------
    #-------------------------------------------------------------------
    robotThreshold = np.float_([threshold])
    session.putvalue('threshold', robotThreshold)

    logging.info('Set robotThreshold completed')
    logging.debug(session.getvalue('threshold'))

    #-------------------------------------------------------------------
    # -----PYTHON: robotRadius, MATLAB: robots SIZE: nx1----------------
    #-------------------------------------------------------------------
    rRadius = []
    for roboName, propRadius in robots.iteritems():
        rRadius.append([propRadius])
    robotRadius = np.float_(rRadius)

    session.putvalue('robots', robotRadius)

    # map to vertical matrix
    robotVerticalScript = "robots = robots'"
    session.putvalue('robotVerticalScript', robotVerticalScript)
    session.run('eval(robotVerticalScript)')

    logging.info('Set robotRadius completed')
    logging.debug("in python: " + str(robotRadius))
    logging.debug("in MATLAB: " + str(session.getvalue('robots')))

    #---------------------------------------------------------------------------------------
    #----- PYTHON: regionVertices, MATLAB: vertices SIZE: cell(#regions)--------------------
    #---------------------------------------------------------------------------------------

    # initialize cell array in MATLAB
    cellArrayScript = 'vertices = cell(' + str(len(regions)) + ',1)'
    session.putvalue('cellArrayScript', cellArrayScript)
    session.run('eval(cellArrayScript)')

    # # send each region vertices to MATLAB
    # code for getting vertices in LTLMoP
    for regionIdx, region in enumerate(regions):  # TODO: self.rfi.regions in LTLMoP and uncomment below
        # logging.debug(regionIdx)
        # pointArray = [y for y in region.getPoints()]
        # pointArray = map(coordmap_map2lab, pointArray)
        # vertices = np.mat(pointArray)
        #------------------------------------------#
        vertices = np.mat([y for y in region.getPoints()])
        #------------------------------------------#
        # logging.debug("regionIdx:" + str(regionIdx) + "vertices:" + str(vertices))
        # time.sleep(10)
        # vertices = np.mat(region).T  # TODO: remove in LTLMoP

        # add tempRegion to MATLAB vertices array
        session.putvalue('region' + str(regionIdx), np.float_(vertices))
        insertRegiontoCellScript = 'vertices{' + str(regionIdx + 1) + '} = region' + str(regionIdx)
        session.putvalue('insertRegiontoCellScript', insertRegiontoCellScript)
        session.run('eval(insertRegiontoCellScript)')

    logging.info('Set region vertices completed')
    for regionIdx, region in enumerate(regions):
        logging.debug(session.getvalue('region' + str(regionIdx)))

    # --------------------------------------------------------------#
    # adding path to MATLAB to run function at a different location #
    # --------------------------------------------------------------#
    pathScript = 'filePath = addpath(\'C:/Users/Catherine/Desktop/Hadas\')'
    session.putvalue('pathScript', pathScript)
    session.run('eval(pathScript)')
    s = session.getvalue('pathScript')

    logging.info('path set to ' + str(s))

    # return matlab session
    return session

def getMATLABVelocity(session, poseDic, next_regIndicesDict):
    """
    pose  = {'rob1':[-1 ,.5],'rob2':[1,1],'rob3':[3.5 , -1]}
    next_regIndices = {'rob1': 2,'rob2':3,'rob3':3}
    """
    #-------------------------------------------------------------------
    #------PYTHON: robotPose, MATLAB: pose  SIZE: n x d-----------------
    #-------------------------------------------------------------------
    pose = []
    for roboName, poseLoc in poseDic.iteritems():
        pose.append(poseLoc[0:2])
    robotPose = np.float_(pose)

    session.putvalue('pose', robotPose)

    # logging.info('Set robotPose completed')
    logging.debug("python:" + str(robotPose))
    logging.debug("MATLAB:" + str(session.getvalue('pose')))

    #-------------------------------------------------------------------
    # -----PYTHON: robotNextRegion, MATLAB: destination SIZE: nx1-------
    #-------------------------------------------------------------------
    next_regIndices = []
    for roboName, next_idx in next_regIndicesDict.iteritems():
        next_regIndices.append(next_idx + 1)  # correct for matlab idx
    robotNextRegion = np.int_([next_regIndices])
    session.putvalue('destination', robotNextRegion)

    # logging.info('Set robotNextRegion completed')
    logging.debug("in python: " + str(robotNextRegion))
    logging.debug("in MATLAB: " + str(session.getvalue('destination')))

    # run initialization function
    # [vx vy changes currentloc]=getvelocity(pose,threshold,vertices, robots, destination);
    getVelocityScript = '[vx, vy, changes, currentLoc] = feval(@getvelocity, pose, threshold, vertices, robots, destination)'
    session.putvalue('getVelocityScript', getVelocityScript)
    session.run('eval(getVelocityScript)')

    # logging.debug('vx = ' + str(session.getvalue('vx')))
    # logging.debug('vy = ' + str(session.getvalue('vy')))

    vx = session.getvalue('vx')
    vy = session.getvalue('vy')
    regionChanges = session.getvalue('changes') - 1
    currentLoc = session.getvalue('currentLoc') - 1

    # return velocities
    return vx, vy, regionChanges, currentLoc

def closeInterface(session):
    """
    close connection with MATLAB
    """
    logging.info('MATLAB session closed')
    del session
