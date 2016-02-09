#LTLRegion.py
"""
This python file contains functions related to region bits and normal region names in LTL.
"""
import ast                  # for parsing spec dict from negtiation monitor
import re                   # for parsing specstr
import parseEnglishToLTL    # for replacing original region name to region bits
import math                 # for adding parentheses around region bits

# logger for ltlmop
import logging
ltlmop_logger = logging.getLogger('ltlmop_logger')

def findRegionBits(ltlFormula, fastslow=False):
    """
    This function finds regions in bits in ltl formula.
    INPUT:
    ltlFormula: ltl formula in normal format
    OUTPUT:
    regionBitsList: list of regionbits strings 
    """
    if fastslow:
        pattern = "\(((!?next\(e.sbit[0-9]\)&?)|(!?e.sbit[0-9]&?))+\)"
    else:
        pattern = "\(((!?next\(s.bit[0-9]\)&?)|(!?s.bit[0-9]&?))+\)"
    regionBitsList = [x.group() for x in re.finditer(pattern,ltlFormula)]

    return regionBitsList
    
def matchRegionNumber(regionBitStr, regions, region_domain, newRegionNameToOld, robotName = '', fastslow=False, include_heading=False, patching=False):
    """
    This function takes in a region bit string and regionList and return the actual region string (with next)
    INPUT:
    regionBitStr: region string in bits
    regions: list of region objects (self.proj.rfi.regions)
    region_domain: region domain from self.proj.rfi.regions, see execute.py
    newRegionNameToOld: dict from new names to old names (made from regionMapping)
    robotName: name of the robot
    OUTPUT:
    targetRegionOrig: the original name of the region 
    """

    ltlmop_logger.log(1,'regionBitStr:' + str(regionBitStr))

    # figure out if the regionBitStr contains "next"
    if 'next' in regionBitStr:
        nextTimeStep = True
    else:
        nextTimeStep = False
    
    # isolate each bit
    if fastslow:
        pattern_bit = "!?(next\()?e.sbit[0-9]\)?"
    else:
        pattern_bit = "!?(next\()?s.bit[0-9]\)?"

    individualBitsList = [x.group() for x in re.finditer(pattern_bit, regionBitStr)]

    regionProps = {}
    # calculate region number
    for bit in individualBitsList:
        bit = bit.replace('next(','').replace(')','')
        if fastslow:
            if '!' not in bit:
                regionProps[bit.replace('e.sbit','regionCompleted_b')] = True
            else:
                regionProps[bit.replace('!e.sbit','regionCompleted_b')] = False
        else:
            if '!' not in bit:
                regionProps[bit.replace('s.bit','region_b')] = True
            else:
                regionProps[bit.replace('!s.bit','region_b')] = False

    ltlmop_logger.log(1,'individualBitsList:' + str(individualBitsList))
    ltlmop_logger.log(1,'regionProps:' + str(regionProps))

    # find region in new name
    targetRegionNew = regions[region_domain.propAssignmentsToNumericValue(regionProps)]

    # find reigon in old name
    targetRegionOrig = newRegionNameToOld[targetRegionNew.name]

    # return region name string to env 
    if nextTimeStep is True:
        if fastslow and include_heading:
            return 'next(e.' + robotName + '_' + targetRegionOrig + '_rc)'
        elif not fastslow and patching and not include_heading:
            return 'next(s.' + robotName + '_' + targetRegionOrig + ')'
        else:
            return 'next(e.' + robotName + '_' + targetRegionOrig + ')'
    else:
        if fastslow and include_heading:
            return 'e.' + robotName + '_' + targetRegionOrig + '_rc'
        elif not fastslow and patching and not include_heading:
            return 's.' + robotName + '_' + targetRegionOrig
        else:
            return 'e.' + robotName + '_' + targetRegionOrig

def replaceSYSbitToENVRobotNameAndRegionName(ltlFormula, regions, region_domain, newRegionNameToOld, robotName = ''):
    """
    This function takes in an ltlFormula with region bits, regionList and newRegionNameToOld, and replace all names to the original ones
    ---> s.bit to e.robotName_regionName <----
    """
    ltlFormulaReplaced = replaceAllRegionBitsToOriginalName(ltlFormula, regions, region_domain, newRegionNameToOld, robotName)
    return ltlFormulaReplaced

def replaceENVsbitToENVRobotNameAndRegionNameRC(ltlFormula, regions, region_domain, newRegionNameToOld, robotName = ''):
    """
    This function takes in an ltlFormula with region bits, regionList and newRegionNameToOld, and replace all names to the original ones
    ---> e.sbit to e.robotName_regionName_rc <---
    """
    ltlFormulaReplaced = replaceAllRegionBitsToOriginalName(ltlFormula, regions, region_domain, newRegionNameToOld, robotName, fastslow = True, include_heading=True)
    return ltlFormulaReplaced

def replaceENVsbitToENVRobotNameAndRegionName(ltlFormula, regions, region_domain, newRegionNameToOld, robotName = ''):
    """
    This function takes in an ltlFormula with region bits, regionList and newRegionNameToOld, and replace all names to the original ones
    ---> e.sbit to e.robotName_regionName <---
    """
    ltlFormulaReplaced = replaceAllRegionBitsToOriginalName(ltlFormula, regions, region_domain, newRegionNameToOld, robotName, fastslow = True)
    return ltlFormulaReplaced

def replaceSYSbitToSYSRobotNameAndRegionName(ltlFormula, regions, region_domain, newRegionNameToOld, robotName = ''):
    """
    This function takes in an ltlFormula with region bits, regionList and newRegionNameToOld, and replace all names to the original ones
    ---> s.bit to s.robotName_regionName <----
    """
    ltlFormulaReplaced = replaceAllRegionBitsToOriginalName(ltlFormula, regions, region_domain, newRegionNameToOld, robotName, fastslow = False, include_heading=False, patching=True)
    return ltlFormulaReplaced

def replaceAllRegionBitsToOriginalName(ltlFormula, regions, region_domain, newRegionNameToOld, robotName = '', fastslow = False, include_heading=False, patching=False):
    """
    This function takes in an ltlFormula with region bits, regionList and newRegionNameToOld, and replace all names to the original ones
    INPUT:
    ltlFormula: ltlFormula in normal form
    regions: list of region objects (self.proj.rfi.regions) in new names
    region_domain: region domain from self.proj.rfi.regions, see execute.py
    newRegionNameToOld: dict from new names to old names (made from regionMapping)
    robotName: name of the robot (optional)
    OUTPUT:
    ltlFormulaReplaced: ltl formula with all regionBits replaced 

    For old format:
    ---> s.bit to e.robotName_regionName <----

    For fastslow (include_heading):
    ---> e.sbit to e.robotName_regionName_rc <---

    For fastslow (only neighbour_robot):
    ---> e.sbit to e.robotName_regionName <---

    For not fastslow, patching and no heading:
    ---> s.bit to s.robotName_regionName <----
    """
    
    # make a copy of the string
    ltlFormulaReplaced = ltlFormula
    ltlmop_logger.log(1,'ltlFormulaReplaced:' + str(ltlFormulaReplaced))

    # find the list of bit regions
    regionBitsList = findRegionBits(ltlFormula, fastslow)
    ltlmop_logger.log(1,'regionBitsList:' + str(regionBitsList))

    for regionBitStr in regionBitsList:
        # find original region name 
        regionName = matchRegionNumber(regionBitStr, regions, region_domain, newRegionNameToOld, robotName, fastslow, include_heading, patching)
        
        # replace region bits to name
        ltlFormulaReplaced = ltlFormulaReplaced.replace(regionBitStr, regionName)

    return ltlFormulaReplaced  
    
def replaceRobotNameWithRegionToBits(ltlFormula, bitEncode, robotName, regionList, fastslow=False, include_heading=False):
    """
    This function takes in an ltlFormula, the bitEncode, robotName and the regionList and returns an ltlFormula with the robotName+ region replaced to region bits.
    INPUT:
    ltlFormula: ltlFormula in normal form
    bitEncode: see function BitEncoding in parseEnglishToLTL.py
    robotName: name of the robot
    regionList: list of original region names
    
    OUTPUT:
    ltlFormulaReplaced: ltl formula with all robotName+region replaced 

    For old format:
    --> e.robotName_regionName to s.bit <----

    For fastslow (include_heading):
    --> e.robotName_regionName_rc to e.sbit <----
    --> e.robotName_regionName to s.bit <----

    For fastslow (only neighbour_robot)
    --> e.robotName_regionName to e.sbit <----
    """
    
    # remove our robot name from the spec
    if fastslow:
        if include_heading:
            for region in regionList:
                ltlFormula = ltlFormula.replace('e.' + robotName + '_' + region + '_rc', 'e.' + region)
                ltlFormula = ltlFormula.replace('e.' + robotName + '_' + region, 's.' + region)
        else:
            for region in regionList:
                ltlFormula = ltlFormula.replace('e.' + robotName + '_' + region, 'e.' + region)
    else:
        ltlFormula = ltlFormula.replace('e.' + robotName + '_', 's.')

    # map region name to bits
    ltlFormulaReplaced  = parseEnglishToLTL.replaceRegionName(ltlFormula, bitEncode, regionList)

    return ltlFormulaReplaced

def replaceBiimplicationBits(ltlFormula, regionList, newRegionNameToOld, robotName, fastslow=True, removeSystemProps=False):
    """
    This function replaces any biimplications of single bits
    removeSystemProps: used in negotiation. Modified to e.region <-> next(e.region)
    """
    if fastslow:
        pattern = "(\(next\(e.sbit[0-9]\)<->next\(s.bit[0-9]\)\)&?)+"
    else:
        pattern = "(\(next\(e.sbit[0-9]\)<->s.bit[0-9]\)&?)+"

    # first make the string to replace
    regionBiimplicationList = []
    for region in [newRegionNameToOld[x.name] for x in regionList]:
        if removeSystemProps:
            regionBiimplicationList.append('(e.'+ robotName + '_' + region +'<->next(e.'+ robotName + '_' + region +'))')
        else:
            regionBiimplicationList.append('(next(e.'+ robotName + '_' + region +'_rc)<->next(s.'+ robotName + '_' + region +'))')

    ltlFormula = re.sub(pattern, "&".join(filter(None, regionBiimplicationList)), ltlFormula)

    return ltlFormula

def addParenthesisToBitsGroupInLTLList(ltlFormulaList, regions):
    """
    This function takes in a list of ltl and add parentesis around bits, (they are removed in checkviolation)
    """
    newLTLList = []
    for ltlStr in ltlFormulaList:
        newLTLList.append(addParenthesisToBitsGroupInLTL(ltlStr, regions))
    return newLTLList

def addParenthesisToBitsGroupInLTL(ltlFormula, regions):
    """
    This function takes in a ltl formula and add parentesis around bits, (they are removed in checkviolation)
    """
    numBits = int(math.ceil(math.log(len(regions),2)))
    sBitPattern = "((!?next\(e.sbit[0-9]\)&?)){"+str(numBits)+"}"
    bitPattern = "((!?next\(s.bit[0-9]\)&?)){"+str(numBits)+"}"
    ltlFormula = ltlFormula.replace(' ','')

    for bitStr in [x.group() for x in re.finditer(sBitPattern,ltlFormula)]:
        ltlFormula = ltlFormula.replace(bitStr, '(' + bitStr + ')')

    for bitStr in [x.group() for x in re.finditer(bitPattern,ltlFormula)]:
        ltlFormula = ltlFormula.replace(bitStr, '(' + bitStr + ')')

    return ltlFormula