import sys, os, shutil, time
import parseFormulaTest

""" ======================================
    LTLcheck.py - LTL violation checking module
    ======================================

    Check for violations of the automaton.
"""

class LTL_Check:

    """
    Obtain .ltl of the current specification and trim the string to include only LTL.
    """
    def __init__(self,path,cur_state,sen_state):
        """Print a parse tree to stdout."""
        self.current_state = cur_state
        self.sensor_state  = sen_state 

        with open(path, 'r') as f:
            read_data = ""
            for line in f:     
                if (not line[0] in ['-','L']) and len(line) > 4 :
                    # len(line) > 4 for removing ( and );
                    read_data += line 
                #read_data = f.read()
        f.closed
        #print read_data
        removed_always_true = read_data.replace("[]<>(TRUE)", "")
        removed_tab = removed_always_true.replace("\t", "")
        removed_nextline = removed_tab.replace("\n", "")
        print  >>sys.__stdout__, "Here's the pure ltl from spec:"
        #print removed_nextline  #read_data
        
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
        tree = parseFormulaTest.parseLTL(removed_nextline)
        #print >>sys.__stdout__,tree
        
        
        self.print_tree(tree,parseFormulaTest.p.terminals) 

        value, negate, next = self.evaluate_subtree(tree, parseFormulaTest.p.terminals)


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
                #if element in x:
                #    return True
                v = v or value
            return v
               
    def evaluate_subtree(self, tree, terminals, level=0, negate = False, next = False, disjunction = False):
        """Print a parse tree to stdout."""
        final_value  = True     # final value to be returned. for conjunction and disjunction operations
        disjunction = None
        implication = None
        if not tree[0] in terminals:
            #print prefix + unicode(tree[0])
            #print tree[0]
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
                #print "evaluating system propositions: " + str(key)
                if negate is True:
                    return not self.current_state.outputs[key], negate, next
                else:
                    return self.current_state.outputs[key], negate, next
                    
            # for environement propositions
            elif "e." in tree[0]:
                #print "negate: " + str(negate)  + " next: " + str(next)
                key = tree[0].replace("e.","")
                #print "evaluating env propositions: " + str(key)
                if negate is True:
                    if next is True:
                        #print "shoudl go to this route"
                        return not self.sensor_state[key], negate, next 
                    else:

                        print key, self.current_state.inputs[key]
                        return not self.current_state.inputs[key], negate, next 
                else:
                    if next is True:
                        return self.sensor_state[key], negate, next
                    else:
                        return self.current_state.inputs[key], negate, next
            
            
            negate_in_loop = negate
            next_in_loop   = next
            node_count = 1
            for x in tree[1:]:
                # skip ltl that does not contain a global operator
                if level == 0 :                  
                    if not self.find_element(x,'GloballyOperator'):
                        print "Skipped this line because there's no global opreator."
                        continue                    
                        
                value, negate_in_loop, next_in_loop = self.evaluate_subtree(x, terminals, level+1, negate_in_loop, next_in_loop, disjunction)
                #print "value: " + str(value) + ", final_value: " + str(final_value)

                
                
                if disjunction is True:
                    final_value = final_value or value
                    #print final_value
                elif disjunction is False::
                    final_value = final_value and value
                    #print final_value
                elif implication is True:
                    if node_count == 1:
                        implication_first = value
                elif implication is False: # biimplication
                
                
                if level == 0:
                    
                    if final_value is False:
                        print "This environement safety assumption is violated."
                        print "------------------------------------------------"
                        print parseFormulaTest.parseLTLTree(x)[0]
                        print "------------------------------------------------"
                        tree = parseFormulaTest.parseLTL(parseFormulaTest.parseLTLTree(x)[0])
                        #print >>sys.__stdout__,tree        
                        self.print_tree(tree,parseFormulaTest.p.terminals)
                        
                    else:
                        print parseFormulaTest.parseLTLTree(x)
                        #print "haha:"+ str(x)
                        print "Expression value: " + str(final_value)
                
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


