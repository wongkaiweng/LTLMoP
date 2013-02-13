import sys, os, shutil, time
import parseFormulaTest

""" ======================================
    LTLcheck.py - LTL violation checking module
    ======================================

    Check for violations of the automaton.
"""

# Debugging stdouts (set to True to print)
debug_implication = False        # print operations relating ->
debug_disjunction = False        # print operations relating to & and |
debug_negate      = False        # print operations relating to !
debug_proposition_values = False # print system and env proposition values
debug_true_ltl           = False # print ltl that are evaluated as true
debug_tree_terminal      = False # print the entire tree in terminal

class LTL_Check:

    """
    Check which ltl statement was violated.
    """
    def __init__(self,path,cur_state,sen_state):
        """
        Obtain .ltl of the current specification and trim the string to include only LTL.
        """
        self.current_state = cur_state
        self.sensor_state  = sen_state 

        with open(path, 'r') as f:
            read_data = self.parse_LTL_to_Tree(f)
        f.closed
        
        #print read_data
        removed_tab = read_data.replace("\t", "")
        removed_nextline = removed_tab.replace("\n", "")
        removed_space = removed_nextline.replace(" ", "")
        removed_always_true = removed_space.replace("&[]<>(TRUE)", "")
        
        
        """
        LTLtree = {'expression':{},'parsedFormula':{}}
        sentenceCount = 0
        for e in removed_nextline.split('&'):
            sentenceCount += 1
            line = str(sentenceCount)
            LTLtree['expression'][line]    = e
            LTLtree['parsedFormula'][line] = parseFormulaTest.parseLTL(e)
               
        for key,value in LTLtree['expression'].iteritems():
            print LTLtree['expression'][key]
            print LTLtree['parsedFormula'][key]
        """
        tree = parseFormulaTest.parseLTL(removed_always_true)
        if debug_tree_terminal == True: 
            print  >>sys.__stdout__, "Here's the ltl of the environment assumptions from spec:"
            #print  >>sys.__stdout__, removed_always_true  #read_data
            print >>sys.__stdout__,tree
            self.print_tree(tree,parseFormulaTest.p.terminals) 

        value, negate, next = self.evaluate_subtree(tree, parseFormulaTest.p.terminals)


    def parse_LTL_to_Tree(self,f):
        """
        Read ltl file from LTLMoP and store the ltl in a string.
        """
        read_data = ""
        for line in f:     
            if (not line[0] in ['-','L']) and len(line) > 4 :                    
                # len(line) > 4 for removing ( and );
                read_data += line            
            elif (line.find("LTLSPEC -- Guarantees") != -1):
                print "Here we skipped the ltl guarantees following."
                return read_data
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
        """Print a parse tree to stdout."""
        final_value  = True     # final value to be returned. for conjunction and disjunction operations
        disjunction  = None
        implication  = None
        negate       = False
        to_negate    = False
        if not tree[0] in terminals:

            """
            if tree[0] == 'UnaryFormula':
                return True
            elif tree[0] == 'Assignment':
                return True
            elif tree[0] == 'GloballyOperator':
                return True
            """    
            
            # check for implication (->)    
            if tree[0] =='Implication':
                implication = True
                
            # check for biimplication (->)      
            elif tree[0] =='Biimplication':
                implication = False
             
            # check for disjunction (or)
            elif tree[0] == "Disjunction":
                disjunction = True
                
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
                    print "key: " + str(key) + " value: " + str(self.current_state.outputs[key])
                    print "evaluating system propositions: " + str(key)

                return int(self.current_state.outputs[key]), negate, next
                    
            # for environement propositions
            elif "e." in tree[0]:                
                key = tree[0].replace("e.","")
                if debug_proposition_values == True:
                    print "negate: " + str(negate)  + " next: " + str(next)
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
                    #print x               
                    if not self.find_element(x,'GloballyOperator'):
                        print "Skipped this line because there's no global opreator."
                        continue                    
                    
                    if self.find_element(x,'FinallyOperator'):
                        print "Skipped this line because this is a liveness assumption."
                        continue 
                value, negate_in_loop, next_in_loop = self.evaluate_subtree(x, terminals, level+1, next_in_loop, disjunction)
                
                # for negating value returned in the ltl
                if negate_in_loop == True:
                    to_negate = True
                
                if to_negate == True:
                    if debug_negate == True:
                        print "in negate in loop " + str(node_count)
                    if node_count == 2:
                        if debug_negate ==True:
                            print "NEGATE here"
                            print x
                        value = not value
                
                # Disjunction(|)  
                if disjunction == True:
                    if debug_disjunction  ==True:
                        print "Disjunction(|)"
                        print "value: " + str(value) + ", final_value: " + str(final_value)
                    final_value = final_value or value
                
                # Conjunction(&)
                elif disjunction == False: 
                    if debug_disjunction == True:                  
                        print "Conjunction(&)"
                        print "value: " + str(value) + ", final_value: " + str(final_value)                   
                    final_value = final_value and value 
                
                # Implication(->)                       
                elif implication == True:
                    if debug_implication == True:
                        print  "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"
                        print "Implication: " + str(implication)
                        print node_count 
                    
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
                        print "###############################################"
                        print "This environement safety assumption is violated."                       
                        print parseFormulaTest.parseLTLTree(x)[0]
                        print "###############################################"
                        tree = parseFormulaTest.parseLTL(parseFormulaTest.parseLTLTree(x)[0])
                        #print >>sys.__stdout__,tree        
                        self.print_tree(tree,parseFormulaTest.p.terminals)

                    else:
                        if debug_true_ltl == True:                        
                            print "-----------------------------------------------"
                            print "Expression value: " + str(final_value)    
                            print parseFormulaTest.parseLTLTree(x)[0]
                            print "-----------------------------------------------"
                        else:
                            pass
          
                node_count += 1
            
            if level == 0:
                while (True):
                    time.sleep(30)                    
                                
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


