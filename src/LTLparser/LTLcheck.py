import sys, os, shutil, time
import parseFormulaTest

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
    def __init__(self,path, LTL2LineNo):
        """
        Obtain .ltl of the current specification and trim the string to include only LTL.
        """
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
            removed_tab = key.replace("\t", "")
            removed_nextline = removed_tab.replace("\n", "")
            removed_space = removed_nextline.replace(" ", "")
            tree = parseFormulaTest.parseLTL(removed_space[:-1])
            # value given is line number. when retrieving structured English, do self.read_spec[value-1]
            self.ltlTree_to_lineNo[str(tree)] = value   
            
        
        # trim read_data so that it only includes ltl but not tabs and nextlines
        read_ltl  = read_ltl.replace("\t", "")
        read_ltl  = read_ltl.replace("\n", "")
        read_ltl  = read_ltl.replace(" ", "")
        read_ltl  = read_ltl.replace("&[]<>(TRUE)", "")    
     
        self.ltl_tree = parseFormulaTest.parseLTL(read_ltl)
        if debug_tree_terminal == True: 
            print  >>sys.__stdout__, "Here's the ltl of the environment assumptions from spec:"
            print >>sys.__stdout__,self.ltl_tree
            self.print_tree(self.ltl_tree,parseFormulaTest.p.terminals) 

        self.violated_spec_line_no = []
            
    def checkViolation(self,cur_state,sensor_state):
        """
        this function call the subtree function to check for violation
        """
        self.current_state = cur_state
        self.sensor_state  = sensor_state
        
        if debug_proposition_values == True:
            print "self.current_state.outputs"
            for key,value in self.current_state.outputs.iteritems():
                print str(key) + ": " + str(value)
            print "self.current_state.inputs"
            for key,value in self.current_state.inputs.iteritems():
                print str(key) + ": " + str(value)
            print "self.sensor_state"              ####SEARCH FOR SELF.SENSOR_STATE TO REMOVE SELF ############
            for key,value in self.sensor_state.iteritems():
                print str(key) + ": " + str(value) 
           
        # check for env violations     
        value, negate, next = self.evaluate_subtree(self.ltl_tree, parseFormulaTest.p.terminals)

        # Environment Violations are removed
        if value == True:
            self.violated_spec_line_no = []
            print "ViolationSolved:"
        
        
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
                    if not self.find_element(x,'GloballyOperator'):
                        if debug_tree_terminal == True:
                            print "Skipped this line because there's no global opreator."
                        continue                    
                    
                    if self.find_element(x,'FinallyOperator'):
                        if  debug_tree_terminal == True:
                            print "Skipped this line because this is a liveness assumption."
                        continue 
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
                
                
                if level == 0:
                    
                    if value == False:
                        treeNo = self.ltlTree_to_lineNo[str(x)]
                        if treeNo not in self.violated_spec_line_no:
                            print "Violation:#######################################"
                            print "Violation:This environement safety assumption is violated."                       
                            
                            print"Violation:line " + str(treeNo) + ": " + self.read_spec[treeNo-1] 
                            self.violated_spec_line_no.append(treeNo)
                            #tree = parseFormulaTest.parseLTL(parseFormulaTest.parseLTLTree(x)[0])     
                            #self.print_tree(tree,parseFormulaTest.p.terminals)

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
        print >>sys.__stdout__, "lala"
    else:
        print >>sys.__stdout__, prefix + unicode(tree[0])
        for x in tree[1:]:
            if indent == 0:
                #pass
                print >>sys.__stdout__, "haha:"+ str(x)
            print___tree(x, terminals, indent+1)       

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


