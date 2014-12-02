#LTLRegion.py
"""
This python file contains functions related to region bits and normal region names in LTL.
"""
import logging
import ast                  # for parsing spec dict from negtiation monitor
import re                   # for parsing specstr
import parseEnglishToLTL    # for replacing original region name to region bits

def findRegionBits(ltlFormula):
    """
    This function finds regions in bits in ltl formula.
    INPUT:
    ltlFormula: ltl formula in normal format
    OUTPUT:
    regionBitsList: list of regionbits strings
    """
    pattern = "\(((!?next\(s.bit[0-9]\)&?)|(!?s.bit[0-9]&?))+\)"
    regionBitsList = [x.group() for x in re.finditer(pattern,ltlFormula)]

    return regionBitsList

def matchRegionNumber(regionBitStr, regions, region_domain, newRegionNameToOld, robotName = ''):
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
    pattern_bit = "!?(next\()?s.bit[0-9]\)?"
    individualBitsList = [x.group() for x in re.finditer(pattern_bit, regionBitStr)]

    regionProps = {}
    # calculate region number
    for bit in individualBitsList:
        bit = bit.replace('next(','').replace(')','')

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
        return 'next(e.' + robotName + '_' + targetRegionOrig + ')'
    else:
        return 'e.' + robotName + '_' + targetRegionOrig

def replaceAllRegionBitsToOriginalName(ltlFormula, regions, region_domain, newRegionNameToOld, robotName = ''):
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

    ---> s.bit to e.robotName_regionName <----

    """

    # make a copy of the string
    ltlFormulaReplaced = ltlFormula

    # find the list of bit regions
    regionBitsList = findRegionBits(ltlFormula)

    for regionBitStr in regionBitsList:
        # find original region name
        regionName = matchRegionNumber(regionBitStr, regions, region_domain, newRegionNameToOld, robotName)

        # replace region bits to name
        ltlFormulaReplaced = ltlFormulaReplaced.replace(regionBitStr, regionName)

    return ltlFormulaReplaced

def replaceRobotNameWithRegionToBits(ltlFormula, bitEncode, robotName, regionList):
    """
    This function takes in an ltlFormula, the bitEncode, robotName and the regionList and returns an ltlFormula with the robotName+ region replaced to region bits.
    INPUT:
    ltlFormula: ltlFormula in normal form
    bitEncode: see function BitEncoding in parseEnglishToLTL.py
    robotName: name of the robot
    regionList: list of original region names

    OUTPUT:
    ltlFormulaReplaced: ltl formula with all robotName+region replaced

    --> e.robotName_regionName to s.bit <----

    """

    # remove our robot name from the spec
    ltlFormula = ltlFormula.replace('e.' + robotName + '_', 's.')

    # map region name to bits
    ltlFormulaReplaced  = parseEnglishToLTL.replaceRegionName(ltlFormula, bitEncode, regionList)

    return ltlFormulaReplaced

def replaceDisjunctedBitstoRegionNames(ltlFormula, regions, newRegionNameToOld, robotName = ''):
    """
    This function takes in an ltlformula and replace all the disjunct bits to region names
    Using for EnvTransSnippets
    regions: list of region objects (self.proj.rfi.regions) in new names
    newRegionNameToOld: dict from new names to old names (made from regionMapping)
    """
    finalLTLList = []
    ltlFormulaList = ltlFormula.split('\n')
    for subLTL in ltlFormulaList:

        patternBit = "(!?s.bit[0-9])+"
        regionBitsList = [x.group() for x in re.finditer(patternBit,subLTL)]
        logging.debug(regionBitsList)

        # replace with self.proj.rfi.regions
        #regions = ['r1','r2','r3','r4']

        regionDisjunctList = []
        for regionBit in regionBitsList:
            # find value of bit we are looking for
            if "!" in regionBit:
                valueOfBit = '0'
                regionBit = regionBit.replace('!','')
            else:
                valueOfBit = '1'

            # find regions to be added to the disjunct list
            regionBit = regionBit.replace("s.bit","")
            for idx, reg in enumerate(regions):
                binaryIdx = "{0:b}".format(idx)
                if len(binaryIdx) < int(regionBit)+1:
                    # length of binart is too short. That means other bits are zero.
                    if '0' == valueOfBit and "e." + robotName + "_" + newRegionNameToOld[reg.name] not in regionDisjunctList:
                        regionDisjunctList.append("e." + robotName + "_" + newRegionNameToOld[reg.name])
                else:
                    if binaryIdx[-int(regionBit)-1] == valueOfBit and "e." + robotName + "_" + newRegionNameToOld[reg.name] not in regionDisjunctList:
                        regionDisjunctList.append("e." + robotName + "_" + newRegionNameToOld[reg.name])

        logging.debug(regionDisjunctList)

        finalLTLList.append(subLTL.replace(" | ".join(regionBitsList)," | ".join(regionDisjunctList)))


    return '\n'.join(finalLTLList)
