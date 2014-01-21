import sys, os, shutil, time
import parseFormulaTest
from numpy import *
import fsa
from collections import OrderedDict
import re

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

class LTL_Check:

    """
    Check which ltl statement was violated.
    """
    def __init__(self,path, LTL2LineNo, spec):
        """
        Obtain .ltl of the current specification and trim the string to include only LTL.
        """
        self.spec          = spec         # split that got split into different parts
        self.path_ltl      = path
        self.current_state = None
        self.sensor_state  = None
        self.LTL2LineNo    = LTL2LineNo    # mapping ltl back to structed English line number       
        
        # obtain ltl from the .ltl file
        with open(self.path_ltl, 'r') as f:
            read_ltl = self.read_LTL_file(f)
        f.closed
        
        # obtain spec from the .spec file
        self.path_spec = path.replace(".ltl",".spec")
        with open(self.path_spec, 'r') as f:
            self.read_spec = self.read_spec_file(f)
        f.closed
        
        self.ltlTree_to_lineNo = {}
        # correspond line numbers in spec to the structure English and the tree converted 
        for key,value in self.LTL2LineNo.iteritems():
            removed_all = key.replace("\t", "").replace("\n", "").replace(" ", "")
            tree = parseFormulaTest.parseLTL(removed_all[:-1])
            # value given is line number. when retrieving structured English, do self.read_spec[value-1]
            self.ltlTree_to_lineNo[str(tree)] = value   
       
        
        # trim read_data so that it only includes ltl but not tabs and nextlines
        read_ltl  = read_ltl.replace("\t", "").replace("\n", "").replace(" ", "").replace("&[]<>(TRUE)", "")    
     
        self.ltl_tree = parseFormulaTest.parseLTL(read_ltl)
        if debug_tree_terminal == True: 
            print  >>sys.__stdout__, "Here's the ltl of the environment assumptions from spec:"
            print >>sys.__stdout__,self.ltl_tree
            self.print_tree(self.ltl_tree,parseFormulaTest.p.terminals) 

        self.violated_spec_line_no = []
        
        self.modify_stage        = 1       # to be used in modify_LTL_file
        self.sameState           = False   # for tracking whether ltl has to be recreated 
        self.liveness_guarantees = ""
        self.last_sys_guarantee  = ""
        self.first_initial_state_added_to_ltl = False
        
        # for storage of LTL assumptions at the three stages
        self.env_safety_assumptions_stage = {"1": "\t\t\t[]((FALSE", "3": "\t\t\t[]((FALSE", "2": "\t\t\t[](FALSE | ("}
        
        # for tracking the liveness assumptions generated
        #self.liveness_generation_count = 0
        #self.sensors = []
        self.sensor_state_len = None       
            
    def checkViolation(self,cur_state,sensor_state):
        """
        this function call the subtree function to check for violation
        """
        self.current_state = cur_state
        self.sensor_state  = sensor_state
         
        # check for env violations     
        value, negate, next = self.evaluate_subtree(self.ltl_tree, parseFormulaTest.p.terminals)


        if debug_proposition_values == True:
            print "self.current_state.outputs"
            for key,value in self.current_state.outputs.iteritems():
                print str(key) + ": " + str(value)
        
        
        # Environment Violations are removed
        if value == True and len(self.violated_spec_line_no) != 0:
            self.violated_spec_line_no = []
            #print "ViolationSolved:"
        
        # return whether the environment assumptions are being violated
        return value
    
    
    def append_state_to_LTL(self, cur_state = None, sensor_state = None):
        """
        append the current state to the dictionary -- env_safety_assumptions_stage
        """    
        if not cur_state == None:    # None: use the stored current_state in the object
            self.current_state = cur_state
        
        if not sensor_state == None: # None: use the stored sensor_state in the object
            self.sensor_state  = sensor_state
        self.sensor_state_len  = len(self.sensor_state)
        
        ########### MODIFICATION STAGE ###############
        # 1 : to add only current inputs
        # 2 : to add current inputs and next inputs
        # 3 : to add current inputs, next inputs, and current outputs
        # This is reset when liveness assumptions are added
        ##############################################
                  
        
        add_ltl = "\t | ("                
                        
        # for the first stage 
        curInputs = fsa.stateToLTL(self.current_state,env_output=True)  
        add_ltl += curInputs          
        # check if the clause of add_ltl already exists in self.env_safety_assumptions_stage["1"]
        if self.env_safety_assumptions_stage["1"].find(add_ltl) == -1 : 
            self.env_safety_assumptions_stage["1"] += add_ltl + ")"                                   
        
        # for the second stage
        state = fsa.FSA_State("sensors_only",self.sensor_state,None,None)
        nextInputs = fsa.stateToLTL(state,use_next=True,env_output=True)
        add_ltl += " & " + nextInputs         
        # check if the clause of add_ltl already exists in self.env_safety_assumptions_stage["2"]
        if self.env_safety_assumptions_stage["3"].find(add_ltl) == -1 : 
            self.env_safety_assumptions_stage["3"] += add_ltl  + ")"     
        
        # for the third stage   
        if self.env_safety_assumptions_stage["2"] == "\t\t\t[](FALSE | (":
            add_ltl3 = "\n\t (" 
        else:
            add_ltl3 = "\n\t | ("       
             
        curOutputs =   fsa.stateToLTL(self.current_state)              
        add_ltl = " ((" + nextInputs +  ") | !(" + curOutputs +  ")) & (" + curInputs    + ")"
        # check if the clause of add_ltl already exists in self.env_safety_assumptions_stage["3"]
        if self.env_safety_assumptions_stage["2"].find(add_ltl) == -1 :                   
            self.env_safety_assumptions_stage["2"] +=  add_ltl3 + add_ltl + ")" 
            #self.env_safety_assumptions_stage["2"] =  self.env_safety_assumptions_stage["3"]  
            #self.env_safety_assumptions_stage["1"] =  self.env_safety_assumptions_stage["3"] 
            
            
    def modify_LTL_file(self):
        """
        Modify spec['EnvTrans'] for runtime verification "learning" and return the new one.
        """
        """
        if self.sameState == False:
                        
            # append the current, next inputs and current outputs to different stages of env saftey assumptions.
            self.append_state_to_LTL()
            
            self.sameState = True
        """
        self.append_state_to_LTL()

        # choosing modify stage to be added
        #print >>sys.__stdout__,"env_safety_assumptions: " + str(self.env_safety_assumptions_stage[str(self.modify_stage)])
        new_env_safety = self.env_safety_assumptions_stage[str(self.modify_stage)]
        new_env_safety  = new_env_safety.replace("\t", "").replace("\n", "").replace(" ", "") #.replace("&[]<>(TRUE)", "") 
        
        new_env_safety  = new_env_safety + "))"   

        self.ltl_tree = parseFormulaTest.parseLTL(new_env_safety)
        #print >>sys.__stdout__,"self.ltl_tree: "+ str(self.ltl_tree)
        #print___tree(self.ltl_tree,parseFormulaTest.p.terminals)
        # remove line 0 as forced to be so that RV violation for [](FALSE .. is printed again)
        try:
            remove_index = self.violated_spec_line_no.index(0)
            del self.violated_spec_line_no[remove_index]
            
        except:
            pass
            #print "no line 0 is found now\n"     

        return self.env_safety_assumptions_stage[str(self.modify_stage)] + ")) &\n"
        
    def read_LTL_file(self,f):
        """
        Read ltl file from LTLMoP and store the ltl in a string.
        """
        read_data = ""
        for line in f:     
            if (not line[0] in ['-','L']) and len(line) > 4 :                    
                # len(line) > 4 for removing ( and );
                read_data += line            
            elif (line.find("LTLSPEC -- Guarantees") != -1):
                #print "Here we skipped the ltl guarantees following."
                return read_data
        return read_data
        
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
    
    
    def print_tree(self,tree, terminals, indent=0):
        """Print a parse tree to stdout."""
        prefix = "    "*indent
        if tree[0] in terminals:
            pass
            #print >>sys.__stdout__, prefix + repr(tree)
            #print >>sys.__stdout__, "lala"
        else:
            print >>sys.__stdout__, prefix + unicode(tree[0])
            for x in tree[1:]:
                if indent == 0:
                    pass
                    #print >>sys.__stdout__, "haha:"+ str(x)
                self.print_tree(x, terminals, indent+1)   
         
          
    def find_element(self,tree,element):
        """
        Return a boolean True value if the element is found in tree or otherwise False
        """
        v = False
        if element in tree[0]:
            return True
        else:       
            for x in tree[1:]:
                value = self.find_element(x,element)
                v = v or value
            return v
               
    def evaluate_subtree(self, tree, terminals, level=0, next = False, disjunction = False):
        """
        Evaluate the parsed tree and yell the environment assumptions violated.
        """
        final_value  = True     # final value to be returned. for conjunction and disjunction operations
        disjunction  = None
        implication  = None
        negate       = False
        to_negate    = False
        #print >>sys.__stdout__,terminals
        #print >>sys.__stdout__,"tree:" + str(tree)
        if not tree[0] in terminals:  
            
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
                if debug_proposition_values == True:
                    print "evaluating system proposition|  key: " + str(key) + " value: " + str(self.current_state.outputs[key])

                return int(self.current_state.outputs[key]), negate, next
                    
            # for environement propositions
            elif "e." in tree[0]:                
                key = tree[0].replace("e.","")
                if debug_proposition_values == True:
                    print " next: " + str(next)
                    print "evaluating env propositions: " + str(key) 

                if next == True:
                    return int(self.sensor_state[key]), negate, False
                else:
                    return int(self.current_state.inputs[key]), negate,  next
            
 
            next_in_loop   = next
            node_count = 1

            for x in tree[1:]:
                # skip ltl that does not contain a global operator
                if level == 0 :  
                    pass               
#                    if not self.find_element(x,'GloballyOperator'):
#                        if debug_tree_terminal == True:
#                            print >>sys.__stdout__,"Skipped this line because there's no global opreator."
#                            print >>sys.__stdout__,x
#                            print >>sys.__stdout__,"level: " + str(level)
#                        continue                    
#                    
#                    if self.find_element(x,'FinallyOperator'):
#                        if  debug_tree_terminal == True:
#                            print >>sys.__stdout__,"Skipped this line because this is a liveness assumption."
#                        continue 
                        
                value, negate_in_loop, next_in_loop = self.evaluate_subtree(x, terminals, level+1, next_in_loop, disjunction)

                
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
                    #print "there's actually some other cases"
                    #print x
                
                #print >>sys.__stdout__,'level:'+ str(level) + ' value:' + str(value)
                if level == 0:
                    
                    if value == False: 
                        try:
                            treeNo = self.ltlTree_to_lineNo[str(x)]
                            
                            if (treeNo not in self.violated_spec_line_no) and treeNo > 0:
                                #print "Violation:#######################################"
                                #print "Violation:This environement safety assumption is violated."                       
                                
                                #print"Violation:line " + str(treeNo) + ": " + self.read_spec[treeNo-1] 
                                self.violated_spec_line_no.append(treeNo)
                                #tree = parseFormulaTest.parseLTL(parseFormulaTest.parseLTLTree(x)[0])     
                                #self.print_tree(tree,parseFormulaTest.p.terminals)
                        except:     
                            if 0 not in self.violated_spec_line_no:                  
                                #print "Violation:RV#######################################"
                                #print "Violation: " + str(parseFormulaTest.parseLTLTree(x)[0]) 
                                #print "Violation:RV#######################################"
                                treeNo = 0
                                self.violated_spec_line_no.append(treeNo)


                        

                    else:
                        if debug_true_ltl == True:                        
                            print "-----------------------------------------------"
                            print "Expression value: " + str(final_value)    
                            print parseFormulaTest.parseLTLTree(x)[0]
                            print "-----------------------------------------------"
                        else:
                            pass
          
                node_count += 1
            
                            
            return final_value, negate, next                 
            
        else:
            return True, negate, next


def print___tree(tree, terminals, indent=0):
    prefix = "    "*indent
    if tree[0] in terminals:
        
        print >>sys.__stdout__, prefix + repr(tree)
        #print >>sys.__stdout__, "lala"
    else:
        print >>sys.__stdout__, prefix + unicode(tree[0])
        for x in tree[1:]:
            if indent == 0:
                #pass
                print >>sys.__stdout__, "haha:"+ str(x)
            print___tree(x, terminals, indent+1)       

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
            
        splitItem = parseFormulaTest.tokenize(item)
        #print splitItem
        tempObj = ""
        tempLine = []
        for index, element in enumerate(splitItem):
            #print element[0]        
            
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
                #print "must be next: " + element[0]
            
            
        
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

            
        splitItem = parseFormulaTest.tokenize(item)
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
                            #print aut.getAnnotatedRegionName(x)
                            #print re.search('\w+\ \((?P<region>\w+)\)', aut.getAnnotatedRegionName(x))
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
                #print "must be next: " + element[0]
            
            
        
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
print___tree(tree,parseFormulaTest.p.terminals)
#evaluate_subtree(tree,parseFormulaTest.p.terminals)
print parseFormulaTest.p.terminals

print parseFormulaTest.parseLTLTree(tree)[0]
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

print___tree(tree,parseFormulaTest.p.terminals)
"""
