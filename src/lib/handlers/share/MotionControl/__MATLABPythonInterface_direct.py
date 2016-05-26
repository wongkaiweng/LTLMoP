import matlab.engine
import StringIO

import numpy as np
import logging
from collections import OrderedDict
import time  # for pause

two_robots = False
experimentInLab = False
if experimentInLab:
    threshold = 100  # 300 # distance for communication?
else:
    threshold = 1000 # 1000

#usingLTLMoP = False

if two_robots:  # in pixels
    robRadius = OrderedDict([('rob2', 0.5), ('rob1', 0.5)])  # used to be 0.5
else:  # in pixels
    robRadius = OrderedDict([('rob3', 20), ('rob2', 20), ('rob1', 20)])  # used to be 0.5
    if experimentInLab:
        robRadius = OrderedDict([('rob1', 0.5), ('rob2', 0.5), ('rob3', 1)])

robots = robRadius

def initializeMATLABPythonCommunication(regions, coordmap_map2lab):
    """
    intialize communication with MATLAB. send robotRadius, threshold and regions.
    regions: self.rfi.regions
    coordmap_map2lab: self.coordmap_map2lab
    """
    logging.info('Starting MATLAB session...')
    eng = matlab.engine.start_matlab()
    #eng.desktop(nargout=0) # start matlab destop
    eng.eval("warning('off','all')", nargout=0) # don't display warning
    out = StringIO.StringIO() # warning
    err = StringIO.StringIO() # error
    logging.info('MATLAB Session initialized ...')
    logging.info('Warning and Error IO initialized ...')

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
    eng.workspace['threshold'] = threshold
    #logging.info('Set threshold completed')
    logging.debug("threshold: {threshold}".format(threshold=eng.workspace['threshold']))

    #-------------------------------------------------------------------
    # -----PYTHON: robotRadius, MATLAB: robots SIZE: nx1----------------
    #-------------------------------------------------------------------
    eng.workspace['robots'] = matlab.double([[x] for x in robots.values()])
    #logging.info('Set robots completed')
    logging.debug("robots: {robots}, size: {robots_size}".format(robots=eng.workspace['robots'], robots_size=eng.workspace['robots'].size))

    #---------------------------------------------------------------------------------------
    #----- PYTHON: regionVertices, MATLAB: vertices SIZE: cell(#regions)--------------------
    #---------------------------------------------------------------------------------------
    # initialize cell array in MATLAB
    cellArrayScript = 'vertices = cell(' + str(len(regions)) + ',1)'
    eng.eval(cellArrayScript, nargout=0)

    # # send each region vertices to MATLAB
    # code for getting vertices in LTLMoP
    for regionIdx, region in enumerate(regions):  # TODO: self.rfi.regions in LTLMoP and uncomment below
        try:
            region.getPoints()
            vertices = matlab.double([[pt[0],pt[1]] for pt in region.getPoints()])
        except:
            logging.warning('Using non-ltlmop format for regions')
            vertices = matlab.double(np.mat(region).T.tolist())  # TODO: remove in LTLMoP

        #logging.debug(vertices)
        # add tempRegion to MATLAB vertices array
        eng.workspace['region' + str(regionIdx)] = vertices
        insertRegiontoCellScript = 'vertices{' + str(regionIdx + 1) + '} = region' + str(regionIdx)
        eng.eval(insertRegiontoCellScript, nargout=0)

    #logging.info('Set region vertices completed')
    for regionIdx, region in enumerate(regions):
        logging.debug("region{idx}: {region_list}".format(idx=regionIdx, region_list=eng.workspace['region' + str(regionIdx)]))

    # --------------------------------------------------------------#
    # adding path to MATLAB to run function at a different location #
    # --------------------------------------------------------------#
    #pathScript = "filePath = addpath(\'/home/Hadas\')"
    #eng.eval(pathScript, nargout=0)
    #logging.info('path set to {filePath}'.format(filePath=eng.workspace['filePath']))

    # return matlab session
    return eng

def getMATLABVelocity(eng, poseDic, next_regIndicesDict, replan):
    """
    pose  = {'rob1':[-1 ,.5],'rob2':[1,1],'rob3':[3.5 , -1]}
    next_regIndices = {'rob1': 2,'rob2':3,'rob3':3}
    replan = True or False
    """
    #-------------------------------------------------------------------
    #------PYTHON: robotPose, MATLAB: pose  SIZE: n x d-----------------
    #-------------------------------------------------------------------
    eng.workspace['pose'] = matlab.double([x.tolist()[0:2] for x in poseDic.values()])
    #logging.info('Set robotPose completed')
    #logging.debug("python:" + str(robotPose))
    logging.debug("pose: {pose}".format(pose=eng.workspace['pose']))

    #-------------------------------------------------------------------
    # -----PYTHON: robotNextRegion, MATLAB: destination SIZE: nx1-------
    #-------------------------------------------------------------------
    eng.workspace['destination'] = matlab.int16([x+1 for x in next_regIndicesDict.values()])
    #logging.info('Set robotNextRegion completed')
    #logging.debug("in python: " + str(robotNextRegion))
    logging.debug("destination: {dest}".format(dest=eng.workspace['destination']))

    #-------------------------------------------------------------------
    # -----PYTHON: replan, MATLAB: replace SIZE: bool-------
    #-------------------------------------------------------------------
    eng.workspace['replan'] =  matlab.int16(replan)
    # logging.debug("in python: " + str(replan))
    logging.debug("replan: {replan}".format(replan=eng.workspace['replan']))

    # run initialization function
    # [vx vy changes currentloc]=getvelocity(pose,threshold,vertices, robots, destination);
    getVelocityScript = '[vx, vy, changes, currentLoc] = feval(@getvelocity, pose, threshold, vertices, robots, destination, replan)'
    eng.eval(getVelocityScript, nargout=0)

    #logging.debug('changes = ' + str(eng.workspace['changes']))
    #logging.debug('currentLoc = ' + str(eng.workspace['currentLoc']))
    vx = [x[0] for x in eng.workspace['vx']]
    vy = [x[0] for x in eng.workspace['vy']]
    if eng.workspace['changes']:
        regionChanges = [x[0]-1.0 for x in eng.workspace['changes']]
    else:
        regionChanges = []
    currentLoc = [x[0]-1.0 for x in eng.workspace['currentLoc']]

    #logging.debug('vx = ' + str(vx))
    #logging.debug('vy = ' + str(vy))
    #logging.debug('regionChanges = ' + str(regionChanges))
    #logging.debug('currentLoc = ' + str(currentLoc))

    # return velocities
    return vx, vy, regionChanges, currentLoc

def closeInterface(eng):
    """
    close connection with MATLAB
    """
    logging.info('MATLAB session closed')
    eng.eval("warning('on','all')", nargout=0) # don't display warning
    eng.quit()

