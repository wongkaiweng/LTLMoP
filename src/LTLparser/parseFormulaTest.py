#!/usr/bin/python
#
# Parse all LTL formulas in a given XML input specification file into XML format.
# Checks sanity of input/output atomic proposition set

import math
import os
import sys
import resource
import subprocess
import signal
from parser import Parser
from re import match
import StringIO

def print_tree(tree, terminals, indent=0):
    """Print a parse tree to stdout."""
    prefix = "    "*indent
    if tree[0] in terminals:
        print prefix + repr(tree)
    else:
        print prefix + unicode(tree[0])
        for x in tree[1:]:
            print_tree(x, terminals, indent+1)
            
# Allocate global parser
p = Parser()

# =====================================================
# Lexer for the PSL formulas
# =====================================================
def tokenize(str):

    res = []
    while str:
        # Ignoring stuff        
        if str[0].isspace() or (str[0]=='\n'):
            str = str[1:]
            continue

        if str.startswith("FALSE"):
            str = str[5:]
            res.append(("FALSE",))
            continue

        if str.startswith("TRUE"):
            str = str[4:]
            res.append(("TRUE",))
            continue

        m = match('[a-zA-Z0-9_.]+', str)
        if m:
            # Special case: ["X","F","U","W","0","1"]
            if m.group(0) in ["U","W","0","1","next"]:
                res.append((m.group(0),))
                str = str[len(m.group(0)):]
            else:
                res.append(('id', m.group(0)))
                str = str[m.end(0):]
            
            continue

        res.append((str[0],))
        str = str[1:]
    return res

# =====================================================
# Simplify the specifications
# =====================================================
def clean_tree(tree):
    """ Cleans a parse Tree, i.e. removes brackets and so on """
    if tree[0] in p.terminals:
        return tree
    if (tree[0]=="Brackets"):
        return clean_tree(tree[2])
    elif (tree[0]=="Implication") and (len(tree)==2):
        return clean_tree(tree[1])
    elif (tree[0]=="Atomic") and (len(tree)==2):
        return clean_tree(tree[1])
    elif (tree[0]=="Conjunction") and (len(tree)==2):
        return clean_tree(tree[1])
    elif (tree[0]=="Biimplication") and (len(tree)==2):
        return clean_tree(tree[1])
    elif (tree[0]=="Disjunction") and (len(tree)==2):
        return clean_tree(tree[1])
    elif (tree[0]=="Xor") and (len(tree)==2):
        return clean_tree(tree[1])
    elif (tree[0]=="BinaryTemporalFormula") and (len(tree)==2):
        return clean_tree(tree[1])
    elif (tree[0]=="UnaryFormula") and (len(tree)==2):
        return clean_tree(tree[1])
    elif (tree[0]=="AtomicFormula"):
        if len(tree)!=2:
            print >>sys.stderr, "Error: AtomicFormula must have length 2"
            sys.exit(1)
        return clean_tree(tree[1])
    elif (tree[0]=="Implication"):
        return [tree[0],clean_tree(tree[1]),clean_tree(tree[3])]
    elif (tree[0]=="Conjunction"):
        return [tree[0],clean_tree(tree[1]),clean_tree(tree[3])]
    elif (tree[0]=="Biimplication"):
        return [tree[0],clean_tree(tree[1]),clean_tree(tree[3])]
    elif (tree[0]=="Disjunction"):
        return [tree[0],clean_tree(tree[1]),clean_tree(tree[3])]
    elif (tree[0]=="Xor"):
        return [tree[0],clean_tree(tree[1]),clean_tree(tree[3])]
    elif (tree[0]=="BinaryTemporalFormula"):
        return [tree[0],clean_tree(tree[1]),clean_tree(tree[2]),clean_tree(tree[3])]
    elif (tree[0]=="UnaryFormula"):
        return [tree[0],clean_tree(tree[1]),clean_tree(tree[2])]
    elif (tree[0]=="BinaryTemporalOperator"):
        # Remove the "superfluous indirection"
        return clean_tree(tree[1])
    elif (tree[0]=="UnaryTemporalOperator"):
        # Remove the "superfluous indirection"
        return clean_tree(tree[1])
    elif (tree[0]=="Assignment"):
        # Flatten "id" case
        A = [tree[0],tree[1][1]]
        A.extend(tree[2:])
        return A
    else:
        A = [tree[0]]
        for x in tree[1:]:
           A.append(clean_tree(x))
        return A

def flatten_as_much_as_possible(tree):
    """ Flattens nested disjunctions/conjunctions """
    # Ground case?
    if len(tree)==1:
        return tree
    if (type(tree)==type("A")) or (type(tree)==type(u"A")): # TODO: How to do this in the way intended?
        return tree
    newTree = []
    for a in tree:
        newTree.append(flatten_as_much_as_possible(a))
    tree = newTree

    # Conjunction
    if (tree[0]=="Conjunction"):
        parts = [tree[0]]
        for a in tree[1:]:
            if a[0]=="Conjunction":
                parts.extend(a[1:])
            else:
                parts.append(a)
        return parts

    # Disjunction
    if (tree[0]=="Disjunction"):
        parts = [tree[0]]
        for a in tree[1:]:
            if a[0]=="Disjunction":
                parts.extend(a[1:])
            else:
                parts.append(a)
        return parts

    # Xor
    if (tree[0]=="Xor"):
        parts = [tree[0]]
        for a in tree[1:]:
            if a[0]=="Xor":
                parts.extend(a[1:])
            else:
                parts.append(a)
        return parts

    # Every other case
    return tree


# =====================================================
# The Parsing function
# =====================================================
def parseLTL(ltlTxt):

    try:
        input = tokenize(ltlTxt)
        tree = p.parse(input)

    except p.ParseErrors, exception:
        for t,e in exception.errors:
            if t[0] == p.EOF:
                print >>sys.stderr, "Formula end not expected here"
                continue

            found = repr(t[0])
            if len(e) == 1:
                print >>sys.stderr, "Error in LTL formula: "+ltlTxt
                print >>sys.stderr, "Expected %s, but found %s " %(repr(e[0]), found)
            else:
                print >>sys.stderr, "Error in LTL formula: "+ltlTxt
                print >>sys.stderr, "Could not parse %s, "%found
                print >>sys.stderr, "Wanted a token of one of the following forms: "+", ".join([ repr(s) for s in e ])
        sys.exit(1)

    # Convert to a tree
    # print tree
    cleaned_tree = clean_tree(tree)
    # print cleaned_tree
    simplified_tree = flatten_as_much_as_possible(cleaned_tree)
    # print simplified_tree
    return simplified_tree

