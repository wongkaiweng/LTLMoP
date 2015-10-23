import sys, os, shutil, time
import LTLFormula
from numpy import *
import fsa
from collections import OrderedDict
import re
import strategy
import logging

""" ======================================
    LTLcheck.py - LTL violation checking module
    ======================================

    Check for violations of the automaton.
"""

# Debugging stdouts (set to True to print)
debug_proposition_values = False # print system and env proposition values
debug_implication = False        # print operations relating ->
debug_disjunction = False        # print operations relating to & and |
debug_negate      = False        # print operations relating to !
debug_true_ltl           = False # print ltl that are evaluated as true
debug_tree_terminal      = False # print the entire tree in terminal


def filterOneRobotViolatedSpec(violatedList, allRobotsList):
    """
    takes in violatedSpecList and return the ones with one-robot-spec removed
    """
    ltlListFiltered, ltlListExcluded = filterSpecList(violatedList, allRobotsList, one_robot_mode=True)
    return ltlListFiltered

def filterRelatedRobotSpec(ltlList, allRobotsList, relatedRobots, myName =""):
    """
    removes ltl involving only rlatedRobots
    """
    ltlListFiltered, ltlListExcluded = filterSpecList(ltlList, allRobotsList, relatedRobots, one_robot_mode=False, myName=myName)
    logging.debug("ltlListExcluded -relatedSpec:" + str(ltlListExcluded))

    return ltlListFiltered


def filterSpecList(ltlList, keyList, keyListMatch=[], one_robot_mode=False, myName=""):
    """
    This function checks if keys in keyList are in each ltl formula
    .If keys found matches keyListMatch, then the formula is excluded. Otherwise, it is kept.
    This function also automatically ignores matches of one key

    keyList: cannot be empty. Should be list of robots coordinating (including myself)
    """
    logging.debug("keyList:" + str(keyList))
    logging.debug("keyListMatch:" + str(keyListMatch))
    logging.debug("myName:" + str(myName))

    ltlKept = []
    ltlExcluded = []
    for ltl in ltlList:
        keyFound = []
        for key in keyList:
            if checkIfKeyInFormula(ltl, key):
                keyFound.append(key)
        #logging.debug("keyFound:" + str(keyFound))
        if len(keyFound) == 1 and one_robot_mode: # we don't keep one robot spec (assuming to be the case of varphi_e^t)
            logging.debug(str(ltl) + " is ignored with only one robot involved")
            ltlExcluded.append(ltl)
            continue
        elif len(keyFound) == 1 and myName in keyFound:
            # we will never remove spec about only ourselves
            ltlKept.append(ltl)
        elif not len(keyListMatch): # don't care. just match everything except spec of one robot
            ltlKept.append(ltl)
        elif set(keyFound).issubset(set(keyListMatch)) and keyFound:
            # if list is provided, and matches, then ignore this ltl.
            #** now exclude subsets as well. also make sure list is not empty
            #logging.debug(str(ltl) + " is ignored with keyMatch")
            ltlExcluded.append(ltl)
            continue
        else:
            ltlKept.append(ltl)

    return ltlKept, ltlExcluded


def checkIfKeyInFormula(ltlFormula, key):
    """
    **This is only for single formula with no LTL conjunction like []() & []()
    return true if key is found in ltlFormula and false otherwise.
    """
    value, __, __, __ = findLTLWithNoKeyInEnvTrans(LTLFormula.parseLTL(ltlFormula), LTLFormula.p.terminals, 0, key, True)

    return value

def excludeSpecFromFormula(specFormula, toExcludeSpecList):
    """
    return a copy of specFromula with spec from toExcludeSpecList removed.
    """
    # first convert formula from formula to list
    specList = ltlStrToList(specFormula)

    # now exclude spec in toExcludeSpecList
    specListExcluded = excludeSpecFromList(specList, toExcludeSpecList)

    return " &\n ".join(specListExcluded)

def excludeSpecFromList(specList, toExcludeSpecList):
    """
    returns a copy of specList with spec in toExcludeSpecList removed.
    """
    # first find out matches in the list
    toRemoveSpec = []
    for toExcludeSpec in toExcludeSpecList:
        toExcludeSpecTree = LTLFormula.parseLTL(toExcludeSpec)
        for spec in specList:
            tree = LTLFormula.parseLTL(spec)
            if tree == toExcludeSpecTree:
                toRemoveSpec.append(spec)
                break
        else:
            logging.error("This spec is not found in the specList:" + str(toExcludeSpec))

    # remove spec from list
    for x in toRemoveSpec:
        specList.remove(x)

    return specList


def removeLTLwithKeyFromEnvTrans(spec, key):
    # return an LTLFomula with all clauses containing s.bit removed
    value, LTLlist, LTLExcludedList, next = findLTLWithNoKeyInEnvTrans(LTLFormula.parseLTL(spec),LTLFormula.p.terminals, 0, key, False)
    return " &\n ".join(LTLlist)

def removeLTLwithoutKeyFromEnvTrans(spec, key):
    # return an LTLFomula with all clauses containing s.bit kept
    value, LTLlist, LTLExcludedList, next = findLTLWithNoKeyInEnvTrans(LTLFormula.parseLTL(spec),LTLFormula.p.terminals, 0, key, True)
    return " &\n ".join(LTLlist)

def separateLTLwithoutEnvPropFromEnvInit(spec):
    """
    Output
    LTLlist: return an LTLFomula with all clauses containing env prop kept
    LTLExcludedList: LTL without any env prop
    """
    value, LTLlist, LTLExcludedList, next = findLTLWithNoKeyInEnvTrans(LTLFormula.parseLTL(spec),LTLFormula.p.terminals, 0, 'e.', True)
    return " &\n ".join(['('+x+')' for x in LTLlist]), " &\n ".join(['('+x+')' for x in LTLExcludedList])

def separateLTLwithNextSystemProps(spec):
    """
    Output
    LTLlist: LTL without next system prop
    LTLExcludedList: LTL with next system prop
    """
    # return an LTLFomula with all clauses containing s.bit removed
    value, LTLlist, LTLExcludedList, next = findLTLWithNoKeyInEnvTrans(LTLFormula.parseLTL(spec),LTLFormula.p.terminals, 0, 's.', False, True, False)
    return " &\n ".join(LTLlist), " &\n ".join(LTLExcludedList)

def ltlStrToList(ltlFormula):
    """
    ltlFormula: any LTL formula
    Returns LTLList that separates all goals
    """
    LTLlist = []
    tree = LTLFormula.parseLTL(ltlFormula)

    for x in tree[1:]:
        LTLlist.append(LTLFormula.treeToString(x))

    return LTLlist

def findLTLWithNoKeyInEnvTrans(tree, terminals, level=0, key = 's.', mode = False, use_next = False, next = False):
    """
    Return an LTLFomula with formulas involving bits removed
    mode: False if see key then the clause is not kept
          True  if see key then the clause is kept
    use_next: use the next mode. look for next(key...)
    """
    final_value  = False     #false for no s.bit LTL. True for having s.bit LTL
    LTLlist = []
    LTLExcludedList = []

    if not tree[0] in terminals or tree[0] in ('FALSE','TRUE'):

        # for system propositions
        if key in tree[0]:
            if not use_next or (use_next and next):
                return True, [], [], False

        # change the next flag
        elif tree[0] == 'NextOperator':
            next = True

        next_in_loop   = next
        for x in tree[1:]:
            # skip ltl that does not contain a global operator
            if level == 0 :
                pass

            value, LTLsubList, LTLExcludedsubList, next_in_loop = findLTLWithNoKeyInEnvTrans(x, terminals, level+1, key, mode, use_next, next_in_loop)

            final_value = final_value or value

            if level == 0:
                if value == mode: # no s.bit in this formula
                    LTLlist.append(LTLFormula.treeToString(x))
                else:
                    LTLExcludedList.append(LTLFormula.treeToString(x))

    return final_value, LTLlist, LTLExcludedList, next


class LTL_Check:

    """
    Check which ltl statement was violated.
    """
    def __init__(self,path, LTL2LineNo, spec, specType='EnvTrans'):
        """
        Obtain .ltl of the current specification and trim the string to include only LTL.
        path is obsoleted. There's no call in the function and can be removed (if all calls to the function are changed)
        """
        self.spec          = spec         # split that got split into different parts
        self.current_state = None
        self.sensor_state  = None
        self.LTL2LineNo    = LTL2LineNo    # mapping ltl back to structed English line number 
        self.ltl_tree      = None      
        self.LTLMoP        = True          # set 'using LTLMoP' as default. To call checkViolation without LTLMoP, use false mode (see checkEnvViolation.py)

        self.ltlTree_to_lineNo = {}
        # correspond line numbers in spec to the structure English and the tree converted 
        for key,value in self.LTL2LineNo.iteritems():
            removed_all = str(key).replace("\t", "").replace("\n", "").replace(" ", "")
            #remove trailing & 
            if "&" == removed_all[-1]:
                removed_all = removed_all[:-1]
            tree = LTLFormula.parseLTL(removed_all)
            # value given is line number. when retrieving structured English, do self.read_spec[value-1]
            self.ltlTree_to_lineNo[LTLFormula.treeToString(tree)] = value

        # save EnvTrans for runtime monitoring
        self.replaceLTLTree(self.spec[specType])

        if debug_tree_terminal == True: 
            logging.debug("Here's the ltl of the environment assumptions from spec:")
            logging.debug(LTLFormula.printTree(self.ltl_tree,LTLFormula.p.terminals))

        self.violated_spec_line_no = [] # storing lineNo found in the violations
        self.violated_specStr = [] # storing all violations LTL
        self.violated_specStr_with_no_specText_match = []   # for storing specStr with no specText match

        self.modify_stage        = 1       # to be used in modify_LTL_file

        # initialize self.env_safety_assumption_stage
        self.env_safety_assumptions_stage = {"1": '', "3": '', "2": ''}

        # initialize lists to store new behaviors observed.
        self.resetEnvCharacterization()

        # for storage of LTL assumptions at the three stages
        self.setOriginalEnvTrans(self.spec[specType].replace('[]', ''))
        #self.env_safety_assumptions_stage = {"1": "\t\t\t[]((FALSE", "3": "\t\t\t[]((FALSE", "2": "\t\t\t[](FALSE | ("}
        
        # if it's not None, the checkViolations will be carried out twice.
        # modify in execute.py so that single envTrans formula can be displayed.
        self.ltl_treeEnvTrans = None

    def setOriginalEnvTrans(self, ltlFormula):
        """
        This function sets the original envtrans to characterize on
        ltlFormula should be without the always '[]' operators
        """
        # first save thte new ltlFormula
        self.env_safety_assumptions = ltlFormula

        # then update the env_safety_assumption_stages as well
        self.updateEnvSafetyAssumptionsStages()

    def updateEnvSafetyAssumptionsStages(self):
        """
        This function updates self.env_safety_assumptions_stage based on self.env_safety_assumptions, self.add_ltl_current_list and self.add_ltl_current_next_list
        """
        # now update the env_safety_assumptions_stage
        if self.add_ltl_current_list: # check if list is empty
            self.env_safety_assumptions_stage["1"] = '[]((' + self.env_safety_assumptions + ')|\n(' + '|\n'.join(filter(None, self.add_ltl_current_list))+ '))\n'
        else:
            self.env_safety_assumptions_stage["1"] = '[](' + self.env_safety_assumptions + ')\n'

        if self.add_ltl_current_next_list:
            self.env_safety_assumptions_stage["3"] = '[]((' + self.env_safety_assumptions + ')|\n(' + '|\n'.join(filter(None, self.add_ltl_current_next_list))+ '))\n'
        else:
            self.env_safety_assumptions_stage["3"] = '[](' + self.env_safety_assumptions + ')\n'

        self.env_safety_assumptions_stage["2"] = self.env_safety_assumptions_stage["3"]

    def replaceLTLTree(self, ltlFormula):
        """
        This function takes in an LTLFormula, parse it into a tree, and replace the existing one.
        """       
        self.ltl_tree = LTLFormula.parseLTL(str(ltlFormula))
        
    def checkViolation(self,cur_state,sensor_state, LTLMoP = True):
        """
        this function call the subtree function to check for violation
        cur_state: state object. see strategy.py
        sensor_state: state object. see strategy.py
        """

        # Set if we are using LTLMoP
        self.LTLMoP = LTLMoP

        self.current_state = cur_state
        self.sensor_state  = sensor_state
        self.violated_specStr = []
        
        # check for env violations     
        value, negate, next = self.evaluate_subtree(self.ltl_tree, LTLFormula.p.terminals, self.violated_spec_line_no)
        
        # for printing original spec violated
        if not self.ltl_treeEnvTrans is None:
            valueEnvTrans, negateEnvTrans, nextEnvTrans = self.evaluate_subtree(self.ltl_treeEnvTrans, LTLFormula.p.terminals, self.violated_spec_line_no, envTransTree = True)
            #logging.debug('self.ltl_tree:' + str(self.ltl_tree))
            #logging.debug('getInputs:' + str(self.sensor_state.getInputs()))
            #logging.debug("valueEnvTrans:" + str(valueEnvTrans))
            #logging.debug("value:" + str(value))
            #logging.debug("self.violated_spec_line_no:" + str(self.violated_spec_line_no))

        if debug_proposition_values == True:
            logging.debug( "self.current_state:")
            logging.debug(self.current_state.getAll(expand_domains = True))        
        
        # Environment Violations are removed
        if value == True and len(self.violated_spec_line_no) != 0:
            self.violated_spec_line_no = []
            self.violated_specStr_with_no_specText_match = []
            self.violated_specStr = []
        
        # return whether the environment assumptions are being violated
        return value

    def resetEnvCharacterization(self):
        """
        This function resets the env characterization clauses stored so that
        the currently the envTrans is just the original one.
        """
        self.add_ltl_current_list = [] #storing current inputs LTLs
        self.add_ltl_current_next_list = [] # storing current and next inputs LTLs

        #also reset violated list
        self.violated_spec_line_no = []
        self.violated_specStr_with_no_specText_match = []
        self.violated_specStr = []

    def append_state_to_LTL_List(self, cur_state=None, sensor_state=None):
        """
        This function stores each new state in the modified envTrans as list instead.
        (To replace append_state_to_LTL in the future)
        """
        if not cur_state == None:    # None: use the stored current_state in the object
            self.current_state = cur_state

        if not sensor_state == None: # None: use the stored sensor_state in the object
            self.sensor_state = sensor_state

        # for current inputs only
        curInputs = self.current_state.getLTLRepresentation(mark_players=True, use_next=False, include_inputs=True, include_outputs=False)
        if not '(' + curInputs + ')' in self.add_ltl_current_list:
            self.add_ltl_current_list.append('(' + curInputs + ')')

        # for current and next inputs
        nextInputs = self.sensor_state.getLTLRepresentation(mark_players=True, use_next=True, include_inputs=True, include_outputs=False)
        if not '(' + curInputs + ' & ' + nextInputs + ')' in self.add_ltl_current_next_list:
            self.add_ltl_current_next_list.append('(' + curInputs + ' & ' + nextInputs + ')')

    def append_state_to_LTL(self, cur_state=None, sensor_state=None):
        """
        append the current state to the dictionary -- env_safety_assumptions_stage
        """
        if not cur_state == None:    # None: use the stored current_state in the object
            self.current_state = cur_state

        if not sensor_state == None: # None: use the stored sensor_state in the object
            self.sensor_state = sensor_state

        ########### MODIFICATION STAGE ###############
        # 1 : to add only current inputs
        # 2 : to add current inputs and next inputs
        # 3 : to add current inputs, next inputs, and current outputs
        # This is reset when liveness assumptions are added
        ##############################################

        # first add the state to our ltl list
        self.append_state_to_LTL_List(cur_state, sensor_state)

        # now update the env_safety_assumptions_stage
        self.updateEnvSafetyAssumptionsStages()

    def updateEnvTransTree(self, originalEnvTrans):
        """
        This function updates the EnvTrans Tree
        """
        # choosing modify stage to be added
        self.ltl_tree = LTLFormula.parseLTL(str(originalEnvTrans + self.env_safety_assumptions_stage[str(self.modify_stage)]))


    def modify_LTL_file(self, originalEnvTrans):
        """
        Modify spec['EnvTrans'] for runtime verification "learning" and return the new one.
        originalEnvTrans: original env safety from user (from structured English to LTL)

        * remember to update the self.ltl_tree with updateEnvTransTree
        """

        self.append_state_to_LTL()

        # choosing modify stage to be added
        #self.ltl_tree = LTLFormula.parseLTL(str(originalEnvTrans + self.env_safety_assumptions_stage[str(self.modify_stage)]))

        # remove line 0 as forced to be so that RV violation for [](FALSE .. is printed again)
        try:
            remove_index = self.violated_spec_line_no.index(0)
            del self.violated_spec_line_no[remove_index]           
        except:
            pass

        return self.env_safety_assumptions_stage[str(self.modify_stage)]


    def read_spec_file(self,f):
        """
        Read spec file from LTLMoP and store the spec in a string.
        """
        read_data = []
        add_line = False
        for line in f:   
            if (line.find("Spec: # Specification in structured English") != -1):
                add_line = True
                continue
             
            if add_line == True:
                read_data.append(line)
                
        return read_data        
               
    def evaluate_subtree(self, tree, terminals,  violated_spec_line_no, level=0, next = False, disjunction = False, envTransTree = False):
        """
        Evaluate the parsed tree and yell the environment assumptions violated.
        violated_spec_line_no
        """
        final_value  = True     # final value to be returned. for conjunction and disjunction operations
        disjunction  = None
        implication  = None
        negate       = False
        to_negate    = False
        if not tree[0] in terminals or tree[0] in ('FALSE','TRUE'):  
            
            # check for implication (->)    
            if tree[0] =='Implication':
                implication = True
                
            # check for biimplication (->)      
            elif tree[0] =='Biimplication':
                implication = False
             
            # check for disjunction (or)
            elif tree[0] == "Disjunction":
                disjunction = True
                final_value = False

            # check for conjunction (and)
            elif tree[0] == "Conjunction":
                disjunction = False
            
            # change the negate flag
            elif tree[0] == 'NotOperator':
                negate = True
                
            # change the next flag 
            elif tree[0] == 'NextOperator':
                next = True
                
            elif tree[0] == 'TRUE':
                return True, negate, False
            
            elif tree[0] == 'FALSE':
                return False, negate, False
                
            # for system propositions
            elif "s." in tree[0]:
                key = tree[0].replace("s.","")
                if not self.LTLMoP:
                    oldKey = key
                if "bit" in key:  #HACK: will be fixed with fsa
                    key = key.replace("bit","region_b")
                if debug_proposition_values == True:
                    print "evaluating system proposition|  key: " + str(key) + " value: " + str(self.current_state[key])
                if self.LTLMoP:
                    return int(self.current_state.getAll(expand_domains=True)[key]), negate, next
                else:
                    if next == True:
                        try:
                            return int(self.sensor_state.getAll(expand_domains=True)[key]), negate, False
                        except:
                            return int(self.sensor_state.getAll(expand_domains=True)[oldKey]), negate, False
                    else:
                        try:
                            return int(self.current_state.getAll(expand_domains=True)[key]), negate, next
                        except:
                            return int(self.current_state.getAll(expand_domains=True)[oldKey]), negate, next

            # for environement propositions
            elif "e." in tree[0]:                
                key = tree[0].replace("e.","")
                if not self.LTLMoP:
                    oldKey = key
                if "sbit" in key:
                    key = key.replace("sbit","regionCompleted_b")
                if debug_proposition_values == True:
                    print " next: " + str(next)
                    print "evaluating env propositions: " + str(key) 

                if next == True:
                    if self.LTLMoP:
                        return int(self.sensor_state.getInputs(expand_domains = True)[key]), negate, False
                    else:
                        try:
                            return int(self.sensor_state.getInputs(expand_domains = True)[key]), negate, False
                        except:
                            return int(self.sensor_state.getInputs(expand_domains = True)[oldKey]), negate, False
                else:
                    if self.LTLMoP:
                        return int(self.current_state.getAll(expand_domains=True)[key]), negate,  next
                    else:
                        try:
                            return int(self.current_state.getAll(expand_domains=True)[key]), negate,  next
                        except:
                            return int(self.current_state.getAll(expand_domains=True)[oldKey]), negate,  next

            next_in_loop   = next
            node_count = 1

            for x in tree[1:]:
                # skip ltl that does not contain a global operator
                if level == 0 :  
                    pass               
                        
                value, negate_in_loop, next_in_loop = self.evaluate_subtree(x, terminals, violated_spec_line_no, level+1, next_in_loop, disjunction)

                
                # for negating value returned in the ltl
                if negate_in_loop == True:
                    to_negate = True
                
                if to_negate == True:
                    if debug_negate == True:
                        print "In negate loop: " + str(node_count)
                    if node_count == 2:
                        if debug_negate ==True:
                            print "NEGATE the value now"
                            print x
                        value = not value
                
                # Disjunction(|)  
                if disjunction == True:
                    if debug_disjunction  ==True:
                        print "Disjunction(|) node: "+ str(node_count) + " value: " + str(value) + ", final_value: " + str(final_value)
                    final_value = final_value or value
                
                # Conjunction(&)
                elif disjunction == False: 
                    if debug_disjunction == True:                  
                        print "Conjunction(&) node: "+ str(node_count) + "value: " + str(value) + ", final_value: " + str(final_value)                   
                    final_value = final_value and value 
                
                # Implication(->)                       
                elif implication == True:
                    if debug_implication == True:
                        print  "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"
                        print "Implication: " + str(node_count)
                    
                    if node_count == 1:
                        implication_first = value
                        
                    else:
                        if debug_implication == True:
                            print "ImplicationElse: Implication_first: " + str(implication_first)
                            print "value: " + str(value) + ", final_value: " + str(final_value)
                            
                        if implication_first == True:
                            if debug_implication == True:                                                          
                                print "Final value  = value"
                                print  "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"
                            final_value = value
                        else:
                            if debug_implication == True: 
                                print "Final value  = True"
                                print  "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"
                            final_value = True
                             
                                               
                # Biimplication (<->)   
                elif implication == False: 
                    if node_count == 1:
                        biimplication_first = value
                    else:
                        final_value = biimplication_first == value
                        
                else:
                    final_value = final_value and value

                if level == 0:
                    
                    if value == False: 
                        try:
                            if not self.LTLMoP:
                                logging.debug("violated line:" +  str(LTLFormula.treeToString(x)))
                                logging.debug(self.ltlTree_to_lineNo)
                            treeNo = self.ltlTree_to_lineNo[LTLFormula.treeToString(x)]
                            if (treeNo not in violated_spec_line_no) and treeNo > 0: 
                                violated_spec_line_no.append(treeNo)
                        except:  
                            #TODO: could be spec from autogeneration but not env characterization 
                            if envTransTree:
                                self.violated_specStr_with_no_specText_match.append(LTLFormula.treeToString(x))
                            else:
                                if 0 not in violated_spec_line_no:                  
                                    treeNo = 0
                                    violated_spec_line_no.append(treeNo)
                        if envTransTree:
                            logging.debug(LTLFormula.treeToString(x))
                            self.violated_specStr.append(LTLFormula.treeToString(x))

                    else:
                        if debug_true_ltl == True:                        
                            print "-----------------------------------------------"
                            print "Expression value: " + str(final_value)    
                            print LTLFormula.treeToString(x)
                            print "-----------------------------------------------"
                        else:
                            pass
          
                node_count += 1
            
                            
            return final_value, negate, next                 
            
        else:
            return True, negate, next    

def parseSlugsEnvTransToNormalEnvTrans(slugsLTLText, sensor_list):
    """
    parse the ltl in slugs format to the normal ltl file format.
    slugsLTLText: ltl in slugs format
    """
    
    CNFclauses = []
    toReturn = ""
    
    for item in slugsLTLText.split('\n'):

        
        if (item.find('#') != -1) or (item.find("SLUGS") != -1) or len(item.replace(" ","")) < 2 :
            #find comment line
            continue
            
        splitItem = LTLFormula.tokenize(item)
        tempObj = ""
        tempLine = []
        for index, element in enumerate(splitItem):
            
            # if the propositions is negated
            if element[0] == '!':
                tempObj += "!"

            # for propositions
            elif element[0] == 'id':  
                if element[1] in sensor_list:
                    obj = "e." + element[1]
                else:
                    obj = "s." + element[1]
                
                try:
                    # figure out if it's next
                    if splitItem[index+1][0] == '\'':
                        tempObj += "next(" + obj + ")"
                    else:
                        tempObj += obj
                except:
                    tempObj += obj
                
                tempLine.append(tempObj)
                tempObj = ""
            else:
                pass
        
        # join each line with "or"
        CNFclauses.append("(" + " | ".join(tempLine) + ")\n")  
        
    # join all the clauses with "and"
    toReturn = "(" + " & ".join(CNFclauses) + ")"
    
    if len(toReturn) == 2:
        return ""        
    else:   
        return "[]" + toReturn 
       
def parseSlugsEnvTransToStructuredEng(slugsLTLText, aut):
    """
    parse the ltl in slugs format to the structured english file format.
    slugsLTLText: ltl in slugs format
    aut : automaton file, for calling self.aut.getAnnotatedRegionName
    """
    
    CNFclauses = []
    toReturn = ""
    
    for item in slugsLTLText.split('\n'):

        
        if (item.find('#') != -1) or (item.find("SLUGS") != -1) or len(item.replace(" ","")) < 2 :
            #find comment line
            continue      

            
        splitItem = treeToString.tokenize(item)
        #print splitItem
        tempObj = ""
        tempLine = []
        for index, element in enumerate(splitItem):
            #print element[0]        
            obj = []
            # if the propositions is negated
            if element[0] == '!':
                tempObj += "not "

            # for propositions
            elif element[0] == 'id':  
                if "bit" in element[1] : #regions
                    bitNo = int(element[1].replace('bit',""))
                    
                    # append regions to the list    
                    for x in range(len(aut.regions)):
                        #figure out if it's a negation of the bit
                        if  tempObj == "not ":
                            checkForRegion = not bool(x & 1 << bitNo)
                        else: 
                            checkForRegion = bool(x & 1 << bitNo)            
                                       
                        if checkForRegion:
                            obj.append(re.search('\w+\ \((?P<region>\w+)\)', aut.getAnnotatedRegionName(x)).group("region"))
                    
                    tempObj = " or ".join(obj)
                            
                else: # other propositions
                    obj.append(element[1])
                
                    try:
                        # figure out if it's next
                        if splitItem[index+1][0] == '\'':
                            tempObj += "next(" + obj[0] + ")"
                        else:
                            tempObj += obj[0]
                    except:
                        tempObj += obj[0]
                
                tempLine.append(tempObj)
                tempObj = ""
            else:
                pass
        
        # join each line with "or"
        CNFclauses.append("(" + " or ".join(tempLine) + ")\n")  
        
    # join all the clauses with "and"
    toReturn = "(" + " and ".join(CNFclauses) + ")"
    
    if len(toReturn) == 2:
        return ""        
    else:   
        return "always" + toReturn 
    

"""
sample = ' []((( ((!s.bit0 & !s.bit1 & !s.bit2)) ) ) -> (   !  next(e.hazardous_item)) ) & []((( ((!s.bit0 & !s.bit1 & !s.bit2)) ) ) -> (   !  next(e.person)) ) '
tree = parseFormulaTest.parseLTL(sample)
print tree
LTLFormula.printTree(tree,LTLFormula.p.terminals)
#evaluate_subtree(tree,parseFormulaTest.p.terminals, violated_spec_line_no)
print parseFormulaTest.p.terminals

print LTLFormula.parseLTL(tree)
print sample
#evaluate_tree('a')
"""
"""
slugsLTLText = "| ! & & ! bit0 ! bit1 ! bit2 | | & & ! bit0' ! bit1' ! bit2' & & ! bit0' bit1' bit2' & & bit0' ! bit1' ! bit2'"
#slugsLTLText = "| ! person' ! & & bit0' ! bit1' ! bit2'"
#slugsLTLText = "| ! & & bit0 ! bit1 ! bit2 | | | & & bit0' ! bit1' ! bit2' & & ! bit0' ! bit1' ! bit2' & & ! bit0' ! bit1' bit2' & & ! bit0' bit1' ! bit2'"
specType = "ENV_TRANS"
ltl =  parseSlugsLTLToNormalLTL(slugsLTLText,specType)
#ltl = "((!(next(bit0))) & (!(next(bit1))) & (!(next(bit2)))) "
for x in ltl:
    tree = parseFormulaTest.parseLTL(x)
print x    
print tree

LTLFormula.printTree(tree,LTLFormula.p.terminals)
"""
