import sys
import re
import time
import pycudd
import strategy

# logger for ltlmop
import logging
ltlmop_logger = logging.getLogger('ltlmop_logger')

# NOTE: This module requires a modified version of pycudd!!
# See src/etc/patches/README_PYCUDD for instructions.

# TODO: We could probably get away with a really minimal Python BDD
#       implementation

# TODO: move generic bdd functions to a different file so others can use?
# TODO: variable reordering so Jx and strat_type are first (is this actually
#       best?)

# TODO: optimize strategy:
#       - stutter state removal
#       - minimal Y after Z change

class BDDStrategy(strategy.Strategy):
    def __init__(self, add=False):
        super(BDDStrategy, self).__init__()

        # We will have a state collection just in order to provide a context
        # for states (FIXME?)
        self.states = strategy.StateCollection()

        self.strategy = None

        self.var_name_to_BDD = {}
        self.BDD_to_var_name = {}

        self.strat_type_var = None

        # track if this is an add or bdd
        self.add = add

        self.mgr = pycudd.DdManager()
        self.mgr.SetDefault()
        # TODO: why is garbage collection crashing?? :( [e.g. on firefighting]
        self.mgr.DisableGarbageCollection()

        self.envTransBDD = None # for saving BDD of envTrans

        #self.mgr.SetMaxMemory(-4000) #<4096
        #print "maxMemory:" + str(self.mgr.ReadMaxMemory())

    def _loadFromFile(self, filename):
        """
        Load in a strategy BDD from a file produced by a synthesizer,
        such as JTLV or Slugs.
        """

        # Clear any existing states
        self.states.clearStates()

        a = pycudd.DdArray(1)

        # Load in the actual BDD or ADD itself
        if self.add:
            # Note: We are using an ADD loader because the BDD loader
            # would expect us to have a reduced BDD with only one leaf node
            self.mgr.AddArrayLoad(pycudd.DDDMP_ROOT_MATCHLIST,
                                  None,
                                  pycudd.DDDMP_VAR_MATCHIDS,
                                  None,
                                  None,
                                  None,
                                  pycudd.DDDMP_MODE_TEXT,
                                  filename, None, a)

            # Convert from a binary (0/1) ADD to a BDD
            self.strategy = self.mgr.addBddPattern(a[0])
            ltlmop_logger.debug('ADD loaded')
        else:
            #try loading as BDD (input from SLUGS) instead of ADD (input from JTLV)
            self.strategy  = self.mgr.BddLoad(pycudd.DDDMP_VAR_MATCHIDS,
                                  None,
                                  None,
                                  None,
                                  pycudd.DDDMP_MODE_TEXT,
                                  filename, None)
            ltlmop_logger.debug('BDD loaded')

        # Load in meta-data
        with open(filename, 'r') as f:
            # Seek forward to the max goal ID notation
            line = ""
            while not line.startswith("# Num goals:"):
                line = f.readline()

            self.num_goals = int(line.split(":")[1])

            # Seek forward to the start of the variable definition section
            while not line.startswith("# Variable names:"):
                line = f.readline()

            # Parse the variable definitions
            for line in f:
                m = re.match(r"^#\s*(?P<num>\d+)\s*:\s*(?P<name>\w+'?)", line)

                # We will stop parsing as soon as we encounter an invalid line
                # Note: This includes empty lines!
                if m is None:
                    break

                varname = m.group("name")
                varnum = int(m.group("num"))

                #### TEMPORARY HACK: REMOVE ME AFTER OTHER COMPONENTS ARE UPDATED!!!
                # Rewrite proposition names to make the old bitvector system work
                # with the new one
                varname = re.sub(r"^bit(\d+)('?)$", r'region_b\1\2', varname)
                varname = re.sub(r"^sbit(\d+)('?)$", r'regionCompleted_b\1\2', varname)
                #################################################################

                if varname == "strat_type":
                    self.strat_type_var = self.mgr.IthVar(varnum)
                else:
                    self.BDD_to_var_name[self.mgr.IthVar(varnum)] = varname
                    self.var_name_to_BDD[varname] = self.mgr.IthVar(varnum)

                # TODO: check for consecutivity

        # Create a Domain for jx to help with conversion to/from bitvectors
        self.jx_domain = strategy.Domain("_jx", value_mapping=range(self.num_goals), endianness=strategy.Domain.B0_IS_LSB)

    def searchForStates(self, prop_assignments, state_list=None, goal_id=None):
        """ Returns an iterator for the subset of all known states (or a subset
            specified in `state_list`) that satisfy `prop_assignments`. """

        if state_list is None:
            state_list_bdd = self.strategy
        else:
            state_list_bdd = self.stateListToBDD(state_list)

        satisfying_state_list_bdd = state_list_bdd & self.propAssignmentToBDD(prop_assignments)

        return self.BDDToStates(satisfying_state_list_bdd)

    def satOne(self, bdd, var_names):
        for vn in var_names:
            test = bdd & ~self.var_name_to_BDD[vn]
            if test:
                bdd = test
            else:
                bdd &= self.var_name_to_BDD[vn]

        return bdd

    def satAll(self, bdd, var_names):
        while bdd:
            one_sat = self.satOne(bdd, var_names)
            yield one_sat
            bdd &= ~one_sat

    def BDDToStates(self, bdd):
        for one_sat in self.satAll(bdd, self.getAllVariableNames() + self.jx_domain.getPropositions()):
            yield self.BDDToState(one_sat)

    def BDDToStatesModified(self, bdd):
        for one_sat in self.satAll(bdd, self.getAllVariableNames() + self.jx_domain.getPropositions()):
            if one_sat & self.strategy:
                yield self.BDDToState(one_sat)

    def BDDToState(self, bdd):
        prop_assignments = self.BDDToPropAssignment(bdd, self.getAllVariableNames())
        jx = self.getJxFromBDD(bdd)

        return self.states.addNewState(prop_assignments, jx)

    def BDDToPropAssignment(self, bdd, var_names):
        prop_assignments = {k: bool(bdd & self.var_name_to_BDD[k]) for k in var_names}

        return prop_assignments

    def printStrategy(self):
        """ Dump the minterm of the strategy BDD.  For debugging only. """

        self.strategy.PrintMinterm()

    def stateListToBDD(self, state_list, use_next=False):
        return reduce(lambda bdd1, bdd2: bdd1 | bdd2,
                      (self.stateToBDD(s, use_next) for s in state_list))

    def propAssignmentToBDD(self, prop_assignments, use_next=False):
        """ Create a BDD that represents the given *binary* proposition
            assignments (expressed as a dictionary from prop_name[str]->prop_val[bool]).
            If `use_next` is True, all variables will be primed. """

        # Expand all domains in the prop assignments since the BDD operates
        # on binary propositions
        prop_assignments = self.states.expandDomainsInPropAssignment(prop_assignments)

        # Start with the BDD for True
        bdd = self.mgr.ReadOne()

        # Add all the proposition values one by one
        for prop_name, prop_value in prop_assignments.iteritems():
            if use_next:
                prop_name += "'"

            if prop_value:
                bdd &= self.var_name_to_BDD[prop_name]
            else:
                bdd &= ~self.var_name_to_BDD[prop_name]

        return bdd

    def stateToBDD(self, state, use_next=False):
        """ Create a BDD that represents the given state.
            If `use_next` is True, all variables will be primed. """

        state_bdd = self.propAssignmentToBDD(state.getAll(expand_domains=True), use_next)

        if use_next is False:
            # We don't currently use jx in the next
            state_bdd &= self.getBDDFromJx(state.goal_id)

        return state_bdd

    def getAllVariableNames(self, use_next=False):
        if use_next:
            return (v+"'" for v in self.states.getPropositions(expand_domains=True))
        else:
            return self.states.getPropositions(expand_domains=True)

    def getAllVariableBDDs(self, use_next=False):
        return (self.var_name_to_BDD[v] for v in self.getAllVariableNames(use_next))

    def prime(self, bdd):
        # TODO: modify support? error check
        return self._BDDSwapVars(bdd, self.getAllVariableBDDs(use_next=False), self.getAllVariableBDDs(use_next=True))

    def unprime(self, bdd):
        return self._BDDSwapVars(bdd, self.getAllVariableBDDs(use_next=True), self.getAllVariableBDDs(use_next=False))

    def _DDArrayFromList(self, elements):
        # We have to do this silly type conversion because we're using a very loosely-wrapped C library
        dd_array = pycudd.DdArray(len(elements))
        for idx, el in enumerate(elements):
            dd_array[idx] = el

        return dd_array

    def _BDDSwapVars(self, bdd, varset1, varset2):
        # Make sure we have an iterator with a len()
        varset1 = list(varset1)
        varset2 = list(varset2)

        assert len(varset1) == len(varset2)

        dd_varset1 = self._DDArrayFromList(varset1)
        dd_varset2 = self._DDArrayFromList(varset2)

        return bdd.SwapVariables(dd_varset1, dd_varset2, len(varset1))

    def findTransitionableStates(self, prop_assignments, from_state=None):
        """ Return a list of states that can be reached from `from_state`
            and satisfy `prop_assignments`.  If `from_state` is omitted,
            the strategy's current state will be used. """

        if from_state is None:
            from_state = self.current_state

        # If possible, move on to the next goal (only possible if current states fulfils current goal)
        candidates = self._getNextStateBDD(from_state, prop_assignments, "Z")
        if candidates:
            candidate_states = list(self.BDDToStates(candidates))
            #ltlmop_logger.debug('candidate_statesZ:' + str(candidate_states))
            for s in candidate_states:
                # add 1 to jx
                s.goal_id = (s.goal_id + 1) % self.num_goals

            return candidate_states

        # If that wasn't possible, try to move closer to the current goal
        candidates = self._getNextStateBDD(from_state, prop_assignments, "Y")
        if candidates:
            #candidate_states = list(self.BDDToStates(candidates))
            #ltlmop_logger.debug('candidate_statesY:' + str(candidate_states))
            return list(self.BDDToStates(candidates))

        # If we've gotten here, something's terribly wrong
        raise RuntimeError("No next state could be found.")

    def findTransitionableNextStates(self, from_state=None):
        """ Return a list of states that can be reached from `from_state`
            and satisfy `prop_assignments`.  If `from_state` is omitted,
            the strategy's current state will be used. """

        if from_state is None:
            from_state = self.current_state

        # Get possible valid next states
        next_state_restrictions = self.propAssignmentToBDD(prop_assignments, use_next=True)
        ltlmop_logger.debug('we did come and check')
        candidates = self.unprime(self.stateToBDD(from_state)
                                  & self.strategy
                                  & self.envTransBDD
                                  & next_state_restrictions)
        ltlmop_logger.debug('we did finish')

        statesToKeep = []
        if candidates:
            candidate_states = list(self.BDDToStates(candidates))
            #candidate_states = list(self.BDDToStatesModified(candidates))
            for state in candidate_states:
                try:
                    state.getAll()
                    statesToKeep.append(state)
                except:
                    pass
                    #ltlmop_logger.debug('State contains invalid assignments. proping state')
                    #ltlmop_logger.debug(state.getAll(expand_domains=True))

            #ltlmop_logger.debug('statesToKeep:' + str(statesToKeep))
            return statesToKeep
        else:
            # If we've gotten here, something's terribly wrong
            ltlmop_logger.error("No next state could be found.")
            return []

    def _getNextStateBDD(self, from_state, prop_assignments, strat_type):
        # Explanation of the strat_type var (from JTLV code):
        #    0. The strategies that do not change the justice pursued
        #    1. The strategies that change the justice pursued
        if strat_type == "Y":
            strat_type_bdd = ~self.strat_type_var
        elif strat_type == "Z":
            strat_type_bdd = self.strat_type_var
        else:
            raise ValueError("Invalid strategy type")

        next_state_restrictions = self.propAssignmentToBDD(prop_assignments, use_next=True)
        candidates = self.unprime(  self.stateToBDD(from_state)
                                  & self.strategy
                                  & strat_type_bdd
                                  & next_state_restrictions)
        return candidates

    def getBDDFromJx(self, jx):
        return self.propAssignmentToBDD(self.jx_domain.numericValueToPropAssignments(jx))

    def getJxFromBDD(self, bdd):
        return self.jx_domain.propAssignmentsToNumericValue(self.BDDToPropAssignment(bdd, self.jx_domain.getPropositions()))

    def evaluateBDD(self, tree, terminals, level=0, next = False, disjunction = False):
        """
        Evaluate the parsed tree and yell the environment assumptions violated.
        violated_spec_line_no
        """

        final_value  = self.mgr.ReadOne()     # final value to be returned. for conjunction and disjunction operations
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
                final_value = self.mgr.ReadZero()

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
                return self.mgr.ReadOne(), negate, False

            elif tree[0] == 'FALSE':
                return self.mgr.ReadZero(), negate, False

            # for system propositions
            elif "s." in tree[0]:
                key = tree[0].replace("s.","")
                if "bit" in key:  #HACK: will be fixed with fsa
                    key = key.replace("bit","region_b")
                if next == True:
                    return self.var_name_to_BDD[key+"'"], negate, False
                else:
                    return self.var_name_to_BDD[key], negate, next

            # for environement propositions
            elif "e." in tree[0]:
                key = tree[0].replace("e.","")
                if "sbit" in key:
                    key = key.replace("sbit","regionCompleted_b")
                if next == True:
                    return self.var_name_to_BDD[key+"'"], negate, False
                else:
                    return self.var_name_to_BDD[key], negate, next

            next_in_loop   = next
            node_count = 1

            for x in tree[1:]:
                # skip ltl that does not contain a global operator
                if level == 0 :
                    pass
                value = None
                ltlmop_logger.debug(x)
                value, negate_in_loop, next_in_loop = self.evaluateBDD(x, terminals, level+1, next_in_loop, disjunction)

                # for negating value returned in the ltl
                if negate_in_loop == True:
                    to_negate = True

                if to_negate == True:
                    if node_count == 2:
                        value = ~value

                # Disjunction(|)
                if disjunction == True:
                    final_value = final_value | value

                # Conjunction(&)
                elif disjunction == False:
                    final_value = final_value & value

                # Implication(->)
                elif implication == True:
                    if node_count == 1:
                        implication_first = value
                    else:
                        final_value = (~implication_first | value)
                        implication_first = None

                # Biimplication (<->)
                elif implication == False:
                    if node_count == 1:
                        biimplication_first = value
                    else:
                        # !!! changed here
                        #final_value = biimplication_first == value
                        final_value = (~biimplication_first | value ) & (~value | biimplication_first)
                        biimplication_first = None
                else:
                    final_value = final_value & value

                if level == 0:
                    pass
                node_count += 1
                value = None

            return final_value, negate, next

        else:
            return self.mgr.ReadOne(), negate, next


