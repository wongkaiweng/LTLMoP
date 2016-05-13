import numpy as np
import logging
ltlmop_logger = logging.getLogger('ltlmop_logger')
import platform
from collections import OrderedDict
from math import sin, cos
import time

import scipy.io as sio
from scipy.interpolate import interp1d
from systemDynamics import Unicycle

system = platform.system()

def initializeController(fname):
    """
    intialize the controller and set up variables
    """

    # Load the Matlab data

    mat_contents = sio.loadmat(fname)

    t_base = 0
    acLastData = [1, 0, 5, 1, [], []]

    numInward = 0 if not len(mat_contents['ac_inward_py']) else len(mat_contents['ac_inward_py'][0])
    numTrans = 0 if not len(mat_contents['ac_trans_py']) else len(mat_contents['ac_trans_py'][0])
    #if (not len(mat_contents['ac_inward'][0]) == numDiscStates):
        #TODO: throw an error!

    # Unpack the mat file 
    state = [mat_contents['aut'][0][0]['state'][0][i][0][0] for i in range(len(mat_contents['aut'][0][0]['state'][0]))]
    label = [mat_contents['aut'][0][0]['label'][0][i][0][0] for i in range(len(mat_contents['aut'][0][0]['label'][0]))]
    trans = [list(mat_contents['aut'][0][0]['trans'][0][i][0]) for i in range(len(mat_contents['aut'][0][0]['trans'][0]))]

    aut = {'state': state,'label': label,'trans': trans}

    ac_inward = [[]]*numInward
    for i in range(numInward):
        print i
        try:
            t = mat_contents['ac_inward_py'][0][i]['t'][0][0][0]
            x0 = mat_contents['ac_inward_py'][0][i]['x0'][0][0]
            u0 = mat_contents['ac_inward_py'][0][i]['u0'][0][0]
            K = mat_contents['ac_inward_py'][0][i]['K'][0][0]
            #Einv = mat_contents['ac_inward_py'][0][i]['Einv'][0][0]['P']
            rho = mat_contents['ac_inward_py'][0][i]['rho'][0][0][0]
            P = mat_contents['ac_inward_py'][0][i]['P'][0][0]
            pre = mat_contents['ac_inward_py'][0][i]['pre'][0][0]
            post = mat_contents['ac_inward_py'][0][i]['post'][0][0]

            ac_inward[i] = {'t':t, 'x0':x0, 'u0':u0, 'K':K, 'rho':rho, 'P':P, 'pre':pre, 'post':post}

        except:
            ac_inward[i] = {'t':[], 'x0':[], 'u0':[], 'K':[], 'rho':[], 'P':[], 'pre':[], 'post':[]}    

    ac_trans = [[]]*numTrans
    for i in range(numTrans):
        try:
            t = mat_contents['ac_trans_py'][0][i]['t'][0][0][0]
            x0 = mat_contents['ac_trans_py'][0][i]['x0'][0][0]
            u0 = mat_contents['ac_trans_py'][0][i]['u0'][0][0]
            K = mat_contents['ac_trans_py'][0][i]['K'][0][0]
            #Einv = mat_contents['ac_trans_py'][0][i]['Einv'][0][0]['P']
            rho = mat_contents['ac_trans_py'][0][i]['rho'][0][0][0]
            P = mat_contents['ac_trans_py'][0][i]['P'][0][0]
            pre = mat_contents['ac_trans_py'][0][i]['pre'][0][0]
            post = mat_contents['ac_trans_py'][0][i]['post'][0][0]

            ac_trans[i] = {'t':t, 'x0':x0, 'u0':u0, 'K':K, 'rho':rho, 'P':P, 'pre':pre, 'post':post}

        except:
            ac_trans[i] = {'t':[], 'x0':[], 'u0':[], 'K':[], 'rho':[], 'P':[], 'pre':[], 'post':[]}

    # Instantiate the system dynamics model
    sysObj = Unicycle()
    
    # Pre-compute a trinary vector for making the point-within-funnel checks are complete
    numCyclicStates = len([i for i, e in enumerate(sysObj.isCyclic) if e != 0])
    vec, cyclicTrinaryVector = buildTrinary([],[[]],numCyclicStates)

    # Make [0, ..., 0] the first element to try
    indexToPop = cyclicTrinaryVector.index([0]*numCyclicStates)
    cyclicTrinaryVector = [[0]*numCyclicStates, cyclicTrinaryVector.pop(indexToPop)]

    # Initialize some other needed data structures TODO: could we simplify?
    data = {'t_offset': [],'t_base': 0,'tend': 0,'t_trials': [],'t_sav': [],'x_sav': [],'x0_sav': [],'t': []}
    acLastData = [1, 0, 5, 1, [], []]

    return sysObj, acLastData, data, aut, ac_trans, ac_inward, cyclicTrinaryVector

def executeController(sysObj, poseDic, regions, curr, next, coordmap_lab2map, scalingPixelsToMeters, doUpdate, acLastData, data, aut, \
    ac_trans, ac_inward, cyclicTrinaryVector, currRegNbr, nextRegNbr, templateIndex, prevTransformationMatrix, transformationMatrix, thetaIndex, inIntersection):
    """

    """

    numRobots = len(poseDic)

    for i, poseLoc in enumerate(poseDic.iteritems()):
        roboName = poseLoc[0]

        #pose = np.float_(np.hstack([float(1)/scalingPixelsToMeters*np.array(coordmap_lab2map(poseLoc[1][0:2])), poseLoc[1][2]]))
        pose = np.float_(np.hstack([float(1)/scalingPixelsToMeters*poseLoc[1][0:2], poseLoc[1][2]]))

        if doUpdate[roboName]:

            currRegName = regions[curr].name
            nextRegName = regions[next].name
            print "current region: "+currRegName
            print "next region: "+nextRegName
            # print regionNumbers
            currNbr = [[]]; nextNbr =[[]]
            for currRegNbr, region in enumerate(regions):
                if region.name == regions[curr].name:
                    break
            for nextRegNbr, region in enumerate(regions):
                if region.name == regions[next].name:
                    break
            # print currRegNbr, nextRegNbr

            print "  currRegNbr: "+str(currRegNbr)
            print "  nextRegNbr: "+str(nextRegNbr) 

    # Execute one step of the local planner and collect velocity components
    vx,vy,w,acLastData,data = executeSingleStep(sysObj,aut,ac_trans,ac_inward,pose,currRegNbr,nextRegNbr,acLastData,data,cyclicTrinaryVector, \
        templateIndex, prevTransformationMatrix, transformationMatrix, thetaIndex, inIntersection)
    # try:
    #     vx,vy,w,acLastData,data = executeSingleStep(sysObj,aut,ac_trans,ac_inward,pose,currRegNbr,nextRegNbr,acLastData,data,cyclicTrinaryVector)
    # except:
    #     print "WARNING: execute function experienced an error!!"

    Vx = {}
    Vy = {}
    W = {}
    for i, poseLoc in enumerate(poseDic.iteritems()):

        Vx[i] = 1*scalingPixelsToMeters*vx
        Vy[i] = 1*scalingPixelsToMeters*vy
        W[i] = 1*w #0.25*w

    # return velocities
    return Vx, Vy, W, acLastData, data, currRegNbr, nextRegNbr


def executeSingleStep(sysObj, aut, ac_trans, ac_inward, x, currReg, nextReg, acLastData, data, cyclicTrinaryVector, templateIndex, prevTransformationMatrix, \
    transformationMatrix, thetaIndex, inIntersection):

    debug = False

    t_offset = data['t_offset']
    t_base = data['t_base']
    tend = data['tend']
    t_trials = data['t_trials']
    t_sav = data['t_sav']
    x_sav = data['x_sav']
    x0_sav = data['x0_sav']
    t = data['t']

    delayTime = -0.1

    timeBaseFlag = False  # if 'true', use time as a basis for choosing the TVLQR states; otherwise use a weighted Euclidean distance.

    phaseWrap = np.array([0, 0, 2*np.pi])

    weights = np.array([1, 1, 0.2])

    m = sysObj.numConfig
    n = sysObj.numState

    #################
    # Account for our manual shifting of the map to the left because of the "dead zone" in the Vicon field 
    #x = x + np.array([0.0043, 0.1629, 0])
    #################

    #################
    # Account for our manual shifting of the map
    #x = x + np.array([-0.9774, 0.0258, 0])
    #################

    #x[2] = x[2] + 0.2  # theta bias needed to account for misalignment of the youBot's coordinate frame wrt. its true orientation

    prevCtrl = False

    if not t_base:
        t_base = time.time() 

    # fail-safe velocity settings
    vx = 0
    vy = 0
    w = 0.001

    teval = 0.02;
    
    # NB: the addtional processing is to correlate to Matlab's indices
    trans = [[i[0]-1, i[1]-1] for i in aut['trans']] # trans contains a list of pairs of states for all transitions in the reduced automaton.
    state = [i-1 for i in aut['state']]  # q are the region labels for each state in the reduced automaton, where the length of q is equal to the total number of states.
    region = [i-1 for i in aut['label']]  # q are the region labels for each state in the reduced automaton, where the length of q is equal to the total number of states.


    # Determine the transition indices as either being an 'intersection' or from matching successor regions to vertices
    # NB: assumes the appropriate order has been stored in the aut...

    if currReg == nextReg:
        u = []
        vx = 0
        vy = 0
        w = 0
        acData = acLastData
    else:

        # find out which template to use based on whether or not we are in an intersection, and the rotation index
        currTransIndices = thetaIndex+1 if inIntersection else 0

        # Also, if not in a funnel yet, we need to apply the old transformation matrix

        ltlmop_logger.debug("Found these possible indices for the current transition: "+str(currTransIndices))

        # next, attempt to match these indices to stored funnels
        acTransIndex = []
        acInwardIndex = []

        currTransIndex = currTransIndices

        # for ii,currTransIndex in enumerate(currTransIndices):
        currState = trans[currTransIndex][0]
        nextState = trans[currTransIndex][1]
        ltlmop_logger.debug("currState (as computed from matching current/next regions): "+str(currState))
        ltlmop_logger.debug("nextState (as computed from matching current/next regions): "+str(nextState))

        for i,ac_test in enumerate(ac_trans):
            # check if there is a stored funnel for this transition
            if ac_test['pre'] == [[currState+1]] and ac_test['post'] == [[nextState+1]]:
                ltlmop_logger.debug('found ac trans for state transition: '+str(ac_test['pre'])+' '+str(ac_test['post']))

                # Check if we are inside this funnel
                print thetaIndex
                print x[2]
                xTmp = (transformationMatrix[:2] * np.mat([x[0], x[1], 1]).T).T
                xTransformed = np.array([xTmp[0,0], xTmp[0,1], x[2] - (thetaIndex-1)*np.pi/2])

                print "inv transformation :"
                print transformationMatrix
                print 'x: '
                print x
                print xTransformed

                if isinternalUnion(ac_test, xTransformed, cyclicTrinaryVector, sysObj):
                    ltlmop_logger.debug("found a transition funnel "+str(i))
                    ac = ac_trans[i]
                    acTransIndex = i
                    break

        if not acTransIndex and ac_inward:
            ltlmop_logger.debug("not inside any transition funnel. attempting to find an inward funnel")
            # for ii,currTransIndex in enumerate(currTransIndices):

            currState = trans[currTransIndex][0]
            ltlmop_logger.debug("currState (as computed from matching current/next regions): "+str(currState))
        
            for i,ac_test in enumerate(ac_inward):

                # check if there is a stored funnel for this state
                if ac_test['pre'] == [[currState+1]]:
                    ltlmop_logger.debug('found ac inward for state: '+str(ac_test['pre']))
            
                    # Check if we are inside this funnel
                    xTmp = (transformationMatrix[:2] * np.mat([x[0], x[1], 1]).T).T
                    xTransformed = np.array([xTmp[0,0], xTmp[0,1], x[2] - (thetaIndex-1)*np.pi/2])

                    print 'x: '
                    print x
                    print xTransformed

                    if isinternalUnion(ac_test, xTransformed, cyclicTrinaryVector, sysObj):
                        ltlmop_logger.debug("found an inward funnel "+str(i))
                        ac = ac_inward[i]
                        acInwardIndex = i
                        break

        ltlmop_logger.debug("acTransIndex"+str(acTransIndex))
        ltlmop_logger.debug("acInwardIndex"+str(acInwardIndex))
        ltlmop_logger.debug('pose: '+str(x))
        
        # if no funnels in the new region/transition OR have temporarily left a funnel, then keep activating the previous one.
        currStateOld = []
        nextStateOld = []
        if not acTransIndex and not acInwardIndex:
            transformationMatrix = prevTransformationMatrix

            acTransIndex = acLastData[0]
            acInwardIndex = acLastData[1]
            ltlmop_logger.debug('acTransIndex (from last iteration): '+str(acTransIndex))
            ltlmop_logger.debug('acInwardIndex (from last iteration): '+str(acInwardIndex))
            if ~debug:
                # # propagate the appropriate old states on to the next iteration
                # if acLastData[4] and acLastData[5]:
                #     currStateOld = acLastData[4]
                #     nextStateOld = acLastData[5]
                # else:
                #     currStateOld = acLastData[2]
                #     nextStateOld = acLastData[3]

                # possibleCurrTrans = [i for i in range(len(trans)) if currStateOld == trans[i][0]]
                # possibleNextTrans = [i for i in range(len(trans)) if nextStateOld == trans[i][1]]
                # currTransOld = [] if not possibleCurrTrans or not possibleNextTrans else list(set(possibleCurrTrans) & set(possibleNextTrans))[0]
                
                # If we were activating a transition funnel previously, then use it; otherwise activate an inward funnel
                if acTransIndex:
                    ac = ac_trans[acTransIndex]
                else:
                    ac = ac_inward[acInwardIndex]
                prevCtrl = True
                ltlmop_logger.debug('WARNING: no funnels found! Using the previous controller.')
                print('WARNING: no funnels found! Using the previous controller.')
            else:
                ac = ac_trans[acLastData[0]]
                ltlmop_logger.debug('WARNING: no funnels found! Forcing an (unverified) controller.')
                print('WARNING: no funnels found! Forcing an (unverified) controller.')
        
        ltlmop_logger.debug('current funnel states: '+str(ac['pre'])+' '+str(ac['post']))
        # determine the new time offset if something has changed
        # TODO: do something smarter here: 
        #       we can remove all the times in the current trajectory prior to the current time, and also possibly add time-weighting to the Euclidean fit
        #       OR - can we do path integration (ref?)

        # This approach computes the command based on a weighted Euclidean distance to the nominal trajectory

        acData = [acTransIndex, acInwardIndex, currState, nextState, currStateOld, nextStateOld]
        
        compareData = []
        for i in range(len(acData)-2):
            compareData.append(acData[i]==acLastData[i])

        acLastData04 = [acLastData[i] for i in range(len(acLastData[0:4])) if acLastData[i]]
        acData04 = [acData[i] for i in range(len(acData[0:4])) if acData[i]]
        acLastData24 = [acLastData[i] for i in range(len(acLastData[2:4])) if acLastData[i]]
        acData24 = [acData[i] for i in range(len(acData[2:4])) if acData[i]]

        if not all(compareData) or not (set(acLastData04) <= set(acData04) and set(acLastData24) == set(acData24)):
            # a change has been detected!!
            t_base = time.time()
            t_trials = ac['t']
        
        # transform the state
        print "transformed state:"
        print transformationMatrix
        print (transformationMatrix[:2] * np.mat([x[0], x[1], 1]).T).T 
        xTmp = (transformationMatrix[:2] * np.mat([x[0], x[1], 1]).T).T 
        print xTmp
        xTransformed = np.array([xTmp[0,0], xTmp[0,1], x[2] - (thetaIndex-1)*np.pi/2])

        minDelta = np.inf
        for i in range(0,len(t_trials),5):
            xtmp = double(ac,'x0',t_trials[i])
            testDist = np.array([
                np.linalg.norm(weights*np.array(sysObj.state2SEconfig(xtmp) - (sysObj.state2SEconfig(xTransformed) + phaseWrap))),
                np.linalg.norm(weights*np.array(sysObj.state2SEconfig(xtmp) - sysObj.state2SEconfig(xTransformed))),
                np.linalg.norm(weights*np.array(sysObj.state2SEconfig(xtmp) - (sysObj.state2SEconfig(xTransformed) - phaseWrap)))])

            testMinDist = min(testDist)

            if (testMinDist < minDelta):
                teval = t_trials[i]
                minDelta = testMinDist
        if not t:  # if time is not given, we compute it here
            t = time.time() - t_base 

        # compute a command
        teval
        K = double(ac,'K',teval) 
        x0 = double(ac,'x0',teval) 
        u0 = double(ac,'u0',teval)
        
        #K = K(end-length(u0)+1:end,:)
        
        u_test = np.array([
                (np.matrix(K)*np.matrix(xTransformed - x0 + sysObj.state2SEconfig(phaseWrap)).T),
                (np.matrix(K)*np.matrix(xTransformed - x0).T),
                (np.matrix(K)*np.matrix(xTransformed - x0 - sysObj.state2SEconfig(phaseWrap)).T)])
        u_idx = list(abs(u_test[:,-1]) == min(abs(u_test[:,-1]))).index([True])
        u_ctrl = u_test[u_idx]

        # Compute the control command
        u = np.array(u0) + np.array([u_ctrl[i][0] for i in range(len(u_ctrl))])
        # u(2) = max(min(u(2),16),-16);

        # Allocate the linear/angular velocities
        vx, vy, w = sysObj.command2robotInput(xTransformed,u)

        #u_sav = [u_sav; t u]
        t_sav = np.append(t_sav, t)
        x_sav = np.append(x_sav, x)
        x0_sav = np.append(x0_sav, x0)

    print "control command: "
    print u, vx, vy, w


    data = {'t_offset': t_offset,'t_base': t_base,'tend': tend,'t_trials': t_trials,'t_sav': t_sav,'x_sav': x_sav,'x0_sav': x0_sav,'t': t}

    return vx, vy, w, acData, data

def isinternalUnion(ac, x, cyclicTrinaryVector, sysObj):

    indexSet = [i for i in range(len(x)) if sysObj.isCyclic[i] > 0]

    xTest = x

    for i, vec in enumerate(cyclicTrinaryVector):
        for k, j in enumerate(indexSet):
            xTest[j] = x[j] + vec[k] * 2*np.pi

        if checkSingleState(ac, xTest):
            return True
    return False

def buildTrinary(vec, dvec, size):

    if len(vec) == size:
        dvec.append(vec[:])
        return vec, dvec

    else:
        vec.append(-1)
        #print vec
        for j in range(-1,2): 
            vec[-1] = j
            # print vec
            vec, dvec = buildTrinary(vec, dvec, size)

        return vec[:-1], dvec

def checkSingleState(ac, x):

    t_test = time.time()
    for i in range(len(ac['t'])):
        q = x - double(ac,'x0',ac['t'][i])
        Qinv = double(ac,'P',ac['t'][i])
        
        # Assume Qinv is not ill-conditioned
        # if i == 0:
        #     print "containment check, time 0: "
        #     print ac['t'][i], x, double(ac,'x0',ac['t'][i]), double(ac,'P',ac['t'][i])
        #     print np.matrix(q) * np.matrix(Qinv) * np.matrix(q).T
        r = float(np.matrix(q) * np.matrix(Qinv) * np.matrix(q).T)
        if (r < 1.) or (abs(r - 1.) < 1e-6):
            #if all(eig(Qinv) >= -1e-6):
            print "time elapsed (checkSingleState): "+str(time.time() - t_test)
            return True
    return False

def double(ac, key, teval):

    t = ac['t']
    x = ac[key]

    if len(x.shape) == 2:
        x1 = []        
        for i in range(len(x)):
            f = interp1d(t,x[i])
            x1.append(float(f(teval)))

    elif len(x.shape) == 3:
        x1 = []
        for i in range(len(x)):
            x1.append([])
            for j in range(len(x[i])):
                f = interp1d(t,x[i][j])
                x1[i].append(float(f(teval)))
    return x1
