import parseFormulaTest
import os , shutil

def print_tree(tree, terminals, indent=0):
    """Print a parse tree to stdout."""
    prefix = "    "*indent
    if tree[0] in terminals:
        print prefix + repr(tree)

    else:
        print prefix + unicode(tree[0])
        for x in tree[1:]:
            if indent == 0:
                print "haha:"+ str(x)
            print_tree(x, terminals, indent+1)
            
def evaluate_tree(path):
    """Print a parse tree to stdout."""
    
    #os.path.join(self.project_root,self.proj.getFilenamePrefix())+".ltl")  # path of ltl file to be passed to the function
    #shutil.copyfile(src, dst)
    
    path = '/home/catherine/LTLMoP/src/examples/firefighting/firefighting.ltl'    

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
    print removed_nextline  #read_data
    
    
    tree = parseFormulaTest.parseLTL(removed_nextline)
    print tree
    
    indent = 0
    terminals = parseFormulaTest.p.terminals
    prefix = "    "*indent
    if tree[0] in terminals:
        print prefix + repr(tree)
    else:
        print prefix + unicode(tree[0])
        for x in tree[1:]:
            print_tree(x, terminals, indent+1)
    
"""   
sample = ' []((( ((!s.bit0 & !s.bit1 & !s.bit2)) ) ) -> (   !  next(e.hazardous_item)) ) & []((( ((!s.bit0 & !s.bit1 & !s.bit2)) ) ) -> (   !  next(e.person)) ) '
tree = parseFormulaTest.parseLTL(sample)
print tree
print_tree(tree,parseFormulaTest.p.terminals)
print parseFormulaTest.p.terminals
"""
evaluate_tree('a')


