import sys, os, shutil, time
import parseFormulaTest
from numpy import *

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
        
        self.modify_stage        = 1       # to be used in modify_LTL_file
        self.sameState           = False   # for tracking whether ltl has to be recreated 
        self.liveness_guarantees = ""
        self.last_sys_guarantee  = ""
        self.first_initial_state_added_to_ltl = False
        
        # for storage of LTL assumptions at the three stages
        self.env_safety_assumptions_stage = {"1": "\t\t\t[](FALSE", "2": "\t\t\t[](FALSE", "3": "\t\t\t[](FALSE"}
        
        # for tracking the liveness assumptions generated
        self.liveness_generation_count = 0
        self.sensors = []
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
            pass
            #print "self.current_state.outputs"
            #for key,value in self.current_state.outputs.iteritems():
            #    print str(key) + ": " + str(value)
        
        """
        #print >>sys.__stdout__,"self.current_state.inputs"
        #for key,value in self.current_state.inputs.iteritems():
        #    print >>sys.__stdout__,"current Inputs: " + str(key) + ": " + str(value)
        #print >>sys.__stdout__,"self.sensor_state" ####SEARCH FOR SELF.SENSOR_STATE TO REMOVE SELF ############
        #for key,value in self.sensor_state.iteritems():
        #    print >>sys.__stdout__,"next Inputs: " + str(key) + ": " + str(value) 
        #print >>sys.__stdout__,"RuntimeV value: " + str(value)
        
        time.sleep(1)
        """
        
        # Environment Violations are removed
        if value == True and len(self.violated_spec_line_no) != 0:
            self.violated_spec_line_no = []
            print "ViolationSolved:"
        
        # return whether the environment assumptions are being violated
        return value
    
    def generate_env_livenss_assumptions(self,initial = False):
        """
        Modify ltl file from LTLMoP to add environment liveness assumptions so that the ltl is synthesizable.
        initial: track if we are this is the first time we run the function or we are trying to find the right liveness.
        """
               
        ########### MODIFICATION STAGE ###############
        # 1 : consider only one current input
        # 2 : consider combination of current inputs
        ##############################################
        
        # reset the count if this is the first time running the function after addition of safety assumptions
        if initial == True:
            self.liveness_generation_count = 0
            
        with open(self.path_ltl, 'r+') as f:
            ltl_file = f.readlines()
            f.seek(0)
            f.truncate()
            liveness_to_add = ""
            for i, line in enumerate(ltl_file):   
                if (ltl_file[i-1].find("[](FALSE") != -1):
                    # for stage 1
                    if self.liveness_generation_count < 2*self.sensor_state_len:
                        # find the sensor to be used in the liveness assumption
                        liveness_to_add += "e." + self.sensors[self.liveness_generation_count/2]
                        
                        # negate the clasue if the count is dividable by 2
                        if self.liveness_generation_count%2 == 0:
                            liveness_to_add = "!" + liveness_to_add
                            
                        liveness_to_add = "\t\t\t[]<>(" + liveness_to_add 
                        self.liveness_generation_count += 1
                   
                    # for stage 2
                    elif self.liveness_generation_count < (2*self.sensor_state_len + 2**self.sensor_state_len): 
                        number = self.liveness_generation_count - 2*self.sensor_state_len
                        sensor_bit = []
                        for i,item in enumerate(self.sensor_state):
                            if i == self.sensor_state_len - 1:
                                #sensor_bit.append(number)
                                if (number == 1):
                                    liveness_to_add += "e." + self.sensors[i] 
                                else:
                                    liveness_to_add += "!e." + self.sensors[i]
                            else:
                                if (number%2 == 1):
                                    liveness_to_add += "e." + self.sensors[i] + " & "
                                else:
                                    liveness_to_add += "!e." + self.sensors[i]+ " & "
                                #sensor_bit.append(number%2)
                                number = number/2
                                                               
                        liveness_to_add = "\t\t\t[]<>(" + liveness_to_add 
                        self.liveness_generation_count += 1
                        
                    else: 
                        print "we don't know why we are here"       
                        pass    
                    
                    print "count: " + str(self.liveness_generation_count) + " adding liveness: "  + str(liveness_to_add)
                    if initial == True:
                        if (line.find("[]<>(TRUE)") != -1):
                            f.write(liveness_to_add+ ") \n")
                        else:
                            f.write(liveness_to_add+ ") &\n" + line)
                    else: # replace the old liveness
                        if (line.find(") &\n") != -1):
                            f.write(liveness_to_add + ") &\n")
                        else:
                            f.write(liveness_to_add + ") \n")
                           
                else: # just rewrite the file
                    f.write(line)        
                                           
        f.closed
        pass
    
    def remove_liveness_guarantees(self):
        """
        Modify ltl file from LTLMoP to remove system liveness guarantees.
        """
             
        with open(self.path_ltl, 'r+') as f:
            ltl_file = f.readlines()
            f.seek(0)
            f.truncate()            
            read_ltl = ""    # for remaking the parse tree of ltl spec
            env_ltl = True
            sys_liveness = False
            
            for i, line in enumerate(ltl_file):   
                #print ltl_file[i-1] + " ,find []<>: " + str(ltl_file[i-1].find("[]<>") != -1)                       
                if (line.find("LTLSPEC -- Guarantees") != -1):
                    env_ltl = False
                
                # when it doesn't find []<> in both this line and previous line
                if (env_ltl == True and not i == 0) :
                    f.write(ltl_file[i-1])
                    #print "writing file normally"
                
                # when it finds []<> in current line
                elif (env_ltl == False and (line.find("[]<>") != -1) and sys_liveness == False):
                    #print "This is the previous line: " + ltl_file[i-1]
                    #print "This is the previous line with no &: " + ltl_file[i-1][:-2]
                    f.write(ltl_file[i-1][:-3])
                    #f.write("\n")
                    sys_liveness = True
                    self.last_sys_guarantee = ltl_file[i-1]
                    self.liveness_guarantees += line
                
                # when storing systme liveness_guarantees
                elif env_ltl == False and sys_liveness == True:
                    #print line + " ,find );: " + str(line.find(");") != -1) 
                    if (line.find(");") == -1):
                        self.liveness_guarantees += line
                    else:
                        self.liveness_guarantees += line # currently this stored ); as well. should just store the entire ltl file. = =''
                        f.write(line)
                
                # when reading system guarantees but not system livenesses yet                  
                else:
                    if env_ltl == False and sys_liveness == False:
                        f.write(ltl_file[i-1])
                    #print "i don't know why it is here"
            #print "self.last_sys_guarantee: " +str(self.last_sys_guarantee)
            #print "liveness_guarantees: " + str(self.liveness_guarantees)
        f.closed
        
        #f.seek(start_of_last_line) 
        #f.write(new_line) # Assuming that new_line ends with "\n" 
        #f.truncate() # In case the new line is shorter than what it's replacing     
        #open(self.path_ltl, 'r+')
        
    def append_liveness_guarantees(self):
        """
        Modify ltl file from LTLMoP to append system liveness guarantees removed .
        """
             
        with open(self.path_ltl, 'r+') as f:
            lines = f.readlines()
            lines = lines[:-1]
            f.seek(0)
            f.truncate()
            for l in lines:   
                f.write(l)
            f.write(self.last_sys_guarantee)
            for l in self.liveness_guarantees:
                f.write(l)
        self.liveness_guarantees = ""
        f.closed
    
    
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
        sensor_state_len_count = 0                 
        #sensor_state_len = len(self.current_state.inputs)
        
        for key,value in self.current_state.inputs.iteritems():
            if not sensor_state_len_count == 0 and sensor_state_len_count < self.sensor_state_len:
                add_ltl += " & "
            if int(value) == False:
                add_ltl += "!"
            add_ltl += "e." + key 
            sensor_state_len_count += 1                      
            #print key, value
            
        # check if the clause of add_ltl already exists in self.env_safety_assumptions_stage["1"]
        if self.env_safety_assumptions_stage["1"].find(add_ltl) == -1 : 
            self.env_safety_assumptions_stage["1"] += add_ltl
            self.env_safety_assumptions_stage["1"] += ")"                        
        
        # for the second stage
        next_sensor_state_len_count = 0 
        #next_sensor_state_len = len(self.sensor_state)
    
        for key,value in self.sensor_state.iteritems():
            if next_sensor_state_len_count == 0 or next_sensor_state_len_count < self.sensor_state_len:
                add_ltl += " & "
            if int(value) == False:
                add_ltl += "!"
            add_ltl += "next(e." + key + ")"
            next_sensor_state_len_count += 1
        
        # check if the clause of add_ltl already exists in self.env_safety_assumptions_stage["2"]
        if self.env_safety_assumptions_stage["2"].find(add_ltl) == -1 : 
            self.env_safety_assumptions_stage["2"] += add_ltl
            self.env_safety_assumptions_stage["2"] += ")"  
        
        # for the third stage                    
        output_state_len_count = 0                             
        output_state_len = len(self.current_state.outputs)
        #print "output_state_len:" + str(output_state_len)
        for key,value in self.current_state.outputs.iteritems():
            if output_state_len_count == 0 or output_state_len_count < output_state_len:
                add_ltl += " & "
            if int(value) == False:
                add_ltl += "!"
            add_ltl += "s." + key
            output_state_len_count += 1
            #print "key: " + str(key) + "value: " + str(value)
            #print output_state_len_count    

        # check if the clause of add_ltl already exists in self.env_safety_assumptions_stage["3"]
        if self.env_safety_assumptions_stage["3"].find(add_ltl) == -1 :                   
            self.env_safety_assumptions_stage["3"] += add_ltl
            self.env_safety_assumptions_stage["3"] += ")"  
        
    
    def modify_LTL_file(self):
        """
        Modify ltl file from LTLMoP for runtime verification "learning".
        """
             
        with open(self.path_ltl, 'r+') as f:
            ltl_file = f.readlines()
            f.seek(0)
            f.truncate()            
            read_ltl = ""    # for remaking the parse tree of ltl spec
            add_assumption = True
            
            for i,line in enumerate(ltl_file):                          
                
                if (line.find("[](FALSE") != -1):
                    
                    if self.sameState == False:
                        
                        # append the current, next inputs and current outputs to different stages of env saftey assumptions.
                        self.append_state_to_LTL()
                        
                        self.sameState = True
                    
                    # choosing modify stage to be added
                    line = self.env_safety_assumptions_stage[str(self.modify_stage)]
                    print "The modification stage is: " + str(self.modify_stage)

                    if self.modify_stage > 3 or self.modify_stage < 1:
                        print "This modify_stage is impossible. stage: " + str (self.modify_stage)
                    #print "STAGE: " + str(self.modify_stage) + " APPENDED: " + str(self.last_added_ltl) + ")" 
                    #line += add_ltl
                    #line += ")) & \n" 
                    line += ") & \n"
                
                """
                # removing the old initial assumption and put in a new one
                if (ltl_file[i-1].find("[](FALSE") != -1):
                    sensor_state_len_count = 0
                    add_init = "("
                    sensor_state_len = len(self.current_state.inputs)
                        
                    for key,value in self.current_state.inputs.iteritems():
                        if not sensor_state_len_count == 0 and sensor_state_len_count < sensor_state_len:
                            add_init += " & "
                        if int(value) == False:
                            add_init += "!"
                        add_init += "e." + key 
                        sensor_state_len_count += 1
                        
                    output_state_len_count = 0    
                    output_state_len = len(self.current_state.outputs)
                    for key,value in self.current_state.outputs.iteritems():
                        if output_state_len_count == 0 or output_state_len_count < output_state_len:
                            add_init += " & "
                        if int(value) == False:
                            add_init += "!"
                        add_init += "s." + key
                        output_state_len_count += 1
                        #print "key: " + str(key) + "value: " + str(value)

                    add_init +=  ") &\n"
                    add_init = "\t\t\t" + add_init
                    #print add_init
                    
                    # To make sure []<>(TRUE) is still in ltl file
                    if self.first_initial_state_added_to_ltl == False:
                        f.write(add_init)
                        self.first_initial_state_added_to_ltl = True
                    else:
                        line = add_init
                """
                
                # find "always TRUE and replace with always FALSE to create most restrictive safety assumptions"
                if (line.find("[](TRUE) &") != -1):
                    line = line.replace("[](TRUE) &", "[](FALSE) &")    
                #print >>sys.__stdout__, line   
                
                f.write(line)
                
                # remaking the parse tree here
                if add_assumption:
                    if (not line[0] in ['-','L']) and len(line) > 4 :                    
                        # len(line) > 4 for removing ( and );
                        read_ltl += line            
                    elif (line.find("LTLSPEC -- Guarantees") != -1):
                        add_assumption = False
                
                
        f.closed
        
        read_ltl  = read_ltl.replace("\t", "")
        read_ltl  = read_ltl.replace("\n", "")
        read_ltl  = read_ltl.replace(" ", "")
        read_ltl  = read_ltl.replace("&[]<>(TRUE)", "")    
     
        self.ltl_tree = parseFormulaTest.parseLTL(read_ltl)
        
        # remove line 0 as forced to be so that RV violation for [](FALSE .. is printed again)
        try:
            remove_index = self.violated_spec_line_no.index(0)
            del self.violated_spec_line_no[remove_index]
            
        except:
            pass
            #print "no line 0 is found now\n"     
        
        
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
        #print >>sys.__stdout__,tree[0]
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
                
                #print >>sys.__stdout__,'level:'+ str(level) + ' value:' + str(value)
                if level == 0:
                    
                    if value == False: 
                        try:
                            treeNo = self.ltlTree_to_lineNo[str(x)]
                            
                            if (treeNo not in self.violated_spec_line_no) and treeNo > 0:
                                print "Violation:#######################################"
                                print "Violation:This environement safety assumption is violated."                       
                                
                                print"Violation:line " + str(treeNo) + ": " + self.read_spec[treeNo-1] 
                                self.violated_spec_line_no.append(treeNo)
                                #tree = parseFormulaTest.parseLTL(parseFormulaTest.parseLTLTree(x)[0])     
                                #self.print_tree(tree,parseFormulaTest.p.terminals)
                        except:     
                            if 0 not in self.violated_spec_line_no:                  
                                print "Violation:RV#######################################"
                                print "Violation: " + str(parseFormulaTest.parseLTLTree(x)[0]) 
                                print "Violation:RV#######################################"
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


