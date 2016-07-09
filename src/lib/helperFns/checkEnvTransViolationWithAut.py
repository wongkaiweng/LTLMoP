import sys
import re
import logging
import ast

import os
p = os.path.abspath(__file__)
print os.path.join(os.path.dirname(p),"../")
sys.path.append(os.path.join(os.path.dirname(p),"../"))

import LTLParser.LTLFormula

import LTLParser.LTLcheck
import fsa
import strategy
import time

logging.basicConfig(level=logging.INFO)

# recursionlimits
logging.debug("recursion limits:" + str(sys.getrecursionlimit()))
sys.setrecursionlimit(10**6)

def getTruePropsInStates(autFile,stateList):
    truePropsOption = True # only returns true proposition if true

    # should feed in a list in ascending order
    try:
        stateCombination = ast.literal_eval(str(stateList))
        stateCombination.reverse()
        listProvided = True

        # pop first element in the list
        state = stateCombination.pop()
    except:
        logging.info('no file given. We will do this for all states.')
        listProvided = False
        stateCombination = []
        state = 0

    f_aut = open(autFile,'r')
    f_output = open('output.txt','w')
    outputDict = {}

    # pattern for detecting propositions
    patternTrueProps = '(?P<propName>\w+):(?P<propValue>[01]),?'

    logging.info('Looking for combination')

    for line in f_aut:
        if 'State '+ str(state) + ' ' in line:
            if truePropsOption:

                trueProps = []
                result = re.finditer(patternTrueProps, line)
                for item in result:
                    #logging.info(item.group('propName') + ":" + item.group('propValue'))
                    if ast.literal_eval(item.group('propValue')):
                        trueProps.append(item.group('propName'))

                patternRank = '(with rank (?P<rankValue>\d+) ->)'
                resultRank = re.finditer(patternRank, line)
                for item in resultRank:
                    #logging.info(item.group('rankValue'))
                    rankValue = item.group('rankValue')
                    f_output.write('State '+ str(state) + ' with rank '+ rankValue +': ' + str(trueProps) +'\n')
                outputDict.update({state:trueProps})

            else:
                f_output.write(line+'\n')
            if len(stateCombination):
                state = stateCombination.pop()
            elif not listProvided:
                state = state + 1
            else:
                break

    logging.info('Output done!')
    f_aut.close()
    f_output.close()
    return outputDict

def checkSuccessors(autFile, allStates=False):
    f_aut = open(autFile,'r')
    state = ''

    noSuccessorsStateList = []
    logging.info('Start checking successors')

    for line in f_aut:
        if 'State' in line:
            state = line
            continue
        else:
            "check successor"
            line = line.replace('With successors :','')
            if re.search('\w+', line) is None or "With no successors." in line or allStates:
                logging.info(state)
                state = state.split('rank')[0]
                stateMatch = re.match(r'State\s(?P<stateNo>\d+)\swith\s',state)
                #stateMatch = re.match(r'State\s(?P<stateNo>\d+)\swith\srank\s(?P<rankNo>\d+\s',state)
                #stateMatch = re.match(r'State\s(?P<stateNo>\d+)\swith\srank\s(?P<rankNo>\d+)\s->\s<(?P<propsList>)>',state)
                noSuccessorsStateList.append(int(stateMatch.group('stateNo')))

    logging.info('Checking done!')
    f_aut.close()

    return noSuccessorsStateList

def getSpecDict(ltlFile):
    assumptions, guarantees = LTLParser.LTLFormula.LTLFormula.fromLTLFile(ltlFile)

    LTLDict = {}
    assumptionsPair = {'EnvInit':LTLParser.LTLFormula.LTLFormulaType.INITIAL,'EnvTrans':LTLParser.LTLFormula.LTLFormulaType.SAFETY,'EnvGoals': LTLParser.LTLFormula.LTLFormulaType.LIVENESS}
    guaranteesPair = {'SysInit':LTLParser.LTLFormula.LTLFormulaType.INITIAL,'SysTrans':LTLParser.LTLFormula.LTLFormulaType.SAFETY,'SysGoals': LTLParser.LTLFormula.LTLFormulaType.LIVENESS}

    for key, formulaType in assumptionsPair.iteritems():
        LTLlist = []
        for x in assumptions.getConjunctsByType(formulaType):
            LTLlist.append(LTLParser.LTLFormula.treeToString(x.tree))

        LTLDict[key]="&".join(LTLlist)

    for key, formulaType in guaranteesPair.iteritems():
        LTLlist = []
        for x in guarantees.getConjunctsByType(formulaType):
            LTLlist.append(LTLParser.LTLFormula.treeToString(x.tree))

        LTLDict[key]="&".join(LTLlist)

    return LTLDict

def getInputsOutputs(smvFile):
    # Read SMV file
    smvFileStream = open(smvFile,"r")
    mode = 0
    inputBits = []
    outputBits = []
    for line in smvFileStream.readlines():
        line = line.strip()
        if (line.startswith("MODULE env")):
            mode = 1
        elif (line.startswith("MODULE sys")):
            mode = 2
        else:
            if line.find(" : ")>0:
                if mode == 1:
                    inputBits.append(line.split(" : ")[0])
                elif mode == 2:
                    outputBits.append(line.split(" : ")[0])
    smvFileStream.close()

    return inputBits, outputBits

def checkViolationWithGivenPropsDict(fileName, propDict):
    """
    This function instead takes in a dict and check if any EnvTrans is violated
    """
    logging.info("Given file name:" + str(fileName))
    logging.info("Given prop dict:" +str(propDict))

    # Then extract assumptions from the LTL file
    spec = getSpecDict(fileName+'.ltl')

    #extract inputs and outputs
    inputs, outputs = getInputsOutputs(fileName+'.smv')

    # create strategy
    strat = strategy.createStrategyFromFile(fileName+'.aut', inputs, outputs)

    # set current state
    currentStateObject = strat.states.addNewState()
    for prop_name, value in propDict.iteritems():
        currentStateObject.setPropValue(prop_name, value)

    # check violation
    LTLViolationCheck = LTLParser.LTLcheck.LTL_Check("",{},spec)
    logging.info('-------------------------')
    logging.info('Printing violations')
    logging.info("EnvTransHolds:" + str(LTLViolationCheck.checkViolation(currentStateObject, currentStateObject, LTLMoP = False)))
    logging.info("Specific line in .spec file:" + str(LTLViolationCheck.violated_specStr))
    logging.info('=========================')


if __name__ == '__main__':
    """
    Example: python extractStates.py '../../examples/firefighting/firefighting'
    This function needs .aut, .ltl and .smv file to work
    look at what is violated for no successor states

    # argv[1] = fileName
    # argv[2] = list of states wanted
    """
    try:
        fileName = sys.argv[1]
    except:
        fileName = '../../examples/firefighting/firefighting'
    logging.info("Given file name:" + str(fileName))

    noSuccessorsStateList = None
    propDict = None

    if len(sys.argv)>2:
        if isinstance(ast.literal_eval(sys.argv[2]),list):
            if not ast.literal_eval(sys.argv[2]): #the list is empty. then check all states.
                noSuccessorsStateList = checkSuccessors(fileName+'.aut', True)
            else:
                noSuccessorsStateList = ast.literal_eval(sys.argv[2])
        elif isinstance(ast.literal_eval(sys.argv[2]),dict):
            propDict = sys.argv[2]
        else:
            logging.error("You have provided a second argument but it's not recognizable!")
            sys.exit()
    else:
        # Find states with no successors and return a list
        noSuccessorsStateList = checkSuccessors(fileName+'.aut')

    if noSuccessorsStateList:
        # extract props from states with no successor
        noSuccessorsStateDict = getTruePropsInStates(fileName+'.aut',noSuccessorsStateList)

        for key in noSuccessorsStateDict.keys():
            logging.info(str(key)+':'+str(noSuccessorsStateDict[key]))

        # Then extract assumptions from the LTL file
        spec = getSpecDict(fileName+'.ltl')

        #extract inputs and outputs
        inputs, outputs = getInputsOutputs(fileName+'.smv')

        # create strategy
        strat = strategy.createStrategyFromFile(fileName+'.aut', inputs, outputs)

        # set up violation check object
        LTLViolationCheck = LTLParser.LTLcheck.LTL_Check("",{},spec)
        LTLViolationCheckSysTrans = LTLParser.LTLcheck.LTL_Check("",{},spec,specType='SysTrans')
        if not spec['SysGoals'].count('[]<>') == 1:
            sysGoalsList  = LTLParser.LTLcheck.ltlStrToList(spec['SysGoals'])
        else:
            sysGoalsList =[spec['SysGoals']]
        LTLViolationCheckSysGoalslist = []
        for ltlStr in sysGoalsList:
            LTLViolationCheckSysGoalslist.append(LTLParser.LTLcheck.LTL_Check("",{},{'SysGoals':ltlStr},specType='SysGoals'))

        winPosSimple = LTLParser.LTLcheck.LTL_Check_slugsWinPos(ltlStr)
        for stateNo in noSuccessorsStateList:
            # find current violated state
            for stateObject in strat.states:
                if stateObject.state_id == str(stateNo):
                    currentStateObject = stateObject

                    # set prop to true if value is not given (In the case of counterstrategy)
                    for prop_name, value in currentStateObject.getAll(expand_domains=True).iteritems():
                        if value is None:
                            currentStateObject.setPropValue(prop_name, False)

                    # now check with the successors
                    for successorStateObject, validTransition in strat.transitions[stateObject].iteritems():
                        if validTransition:
                            # set prop to true if value is not given (In the case of counterstrategy)
                            for prop_name, value in successorStateObject.getAll(expand_domains=True).iteritems():
                                if value is None:
                                    successorStateObject.setPropValue(prop_name, False)

                        # check violation
                        logging.info('-------------------------')
                        logging.info('Printing violations of State ' + str(stateNo) + ' and transition to State ' + str(successorStateObject.state_id))
                        logging.info('State props:' + str([k for k, v in currentStateObject.getAll(expand_domains=True).iteritems() if v]))
                        logging.info('Next State props:' + str([k for k, v in successorStateObject.getAll(expand_domains=True).iteritems() if v]))
                        logging.info("EnvTransHolds:" + str(LTLViolationCheck.checkViolation(currentStateObject, successorStateObject, LTLMoP = False)))
                        logging.info("Specific line in .spec file:" + str(LTLViolationCheck.violated_specStr))
                        logging.info("SysTransHolds:" + str(LTLViolationCheckSysTrans.checkViolation(currentStateObject, successorStateObject, LTLMoP = False)))
                        logging.info("Specific line in .spec file:" + str(LTLViolationCheckSysTrans.violated_specStr))

                    # old for centralized code
                    if 'centralizedSpec' in fileName:
                        for idx, checkObject in enumerate(LTLViolationCheckSysGoalslist):
                            startTime = time.time()
                            if checkObject.checkViolation(currentStateObject, currentStateObject, LTLMoP = False):
                                logging.info(str(idx) + "-SysGoalsHolds: True, Time taken = " + str(time.time() - startTime))
                        logging.info('=========================')
                        startTime = time.time()
                        logging.info('winPosValue:' + str(winPosSimple.checkViolation(currentStateObject, currentStateObject)) + ' , time taken:' + str(time.time() - startTime))

                    break

    elif propDict:
        checkViolationWithGivenPropsDict(fileName, currentStateObject.getAll(expand_domains=True))
