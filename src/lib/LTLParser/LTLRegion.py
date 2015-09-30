#LTLRegion.py
"""
This python file contains functions related to region bits and normal region names in LTL.
"""
import logging
import ast                  # for parsing spec dict from negtiation monitor
import re                   # for parsing specstr
import parseEnglishToLTL    # for replacing original region name to region bits

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

    # find the list of bit regions
    regionBitsList = findRegionBits(ltlFormula, fastslow)
    
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
                
    
    
