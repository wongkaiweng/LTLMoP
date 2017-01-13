#! /usr/bin/env python
import argparse
import re, Polygon, Polygon.IO
import sys, os
p = os.path.abspath(__file__)
sys.path.append(os.path.join(os.path.dirname(p), "../"))
import cairo, rsvg
import itertools
import math

import regions


def region2svg(regionFile, png_file):
    """
    Converts region file to svg
    This is from the deprecated regions file with slight changes for
    proper calculation of the size of the regions map
    """
    rfi = regions.RegionFileInterface()
    rfi.readFile(regionFile)

    fout_list = []

    two_region_list = itertools.combinations(rfi.regions, 2)
    for (region_1, region_2) in two_region_list:
        if not (region_1.name == 'boundary' or region_2.name == 'boundary' \
                or region_1.isObstacle or region_2.isObstacle) \
                and rfi.transitions[rfi.regions.index(region_1)][rfi.regions.index(region_2)]: #adjacent region
            print region_1, region_2

            fout= png_file.replace(".png","_"+region_1.name+"_"+region_2.name+".svg")

            # combine two regions next to each other
            region_1_poly = Polygon.Polygon([(pt.x,-pt.y) for pt in region_1.getPoints()])
            region_2_poly = Polygon.Polygon([(pt.x,-pt.y) for pt in region_2.getPoints()])
            two_region_poly = region_1_poly + region_2_poly

            polyList = [two_region_poly]

            width = 0
            height = 0
            for region in rfi.regions:
                width = max(width, *[pt.x for pt in region.getPoints()])
                height = max(height, *[pt.y for pt in region.getPoints()])

                if region.name != "boundary" and region.name != region_1.name and region.name != region_2.name:
                    points = [(pt.x,-pt.y) for pt in region.getPoints()]
                    poly = Polygon.Polygon(points)
                    polyList.append(poly)

            bound_zero = Polygon.Polygon([(0,0),(1, 0), (1, -1), (0, -1)])
            bound_width = Polygon.Polygon([(width,0),(width-1, 0), (width-1, -1), (width, -1)])
            bound_height = Polygon.Polygon([(0,-height),(-1, -height), (-1, -height+1), (0, -height+1)])
            bound_all = Polygon.Polygon([(width,-height),(width-1, -height), (width-1, -height+1), (width, -height+1)])

            polyList.extend([bound_zero, bound_width, bound_height, bound_all])

            #use boundary size for image size
            Polygon.IO.writeSVG(fout, polyList, width=width,height=height, fill_color=("(255,255,255)",))

            fout_list.append(fout) #return the file name

    # original only
    polyList = []
    fout= png_file.replace(".png", ".svg")

    width = 0
    height = 0
    for region in rfi.regions:
        width = max(width, *[pt.x for pt in region.getPoints()])
        height = max(height, *[pt.y for pt in region.getPoints()])

        if region.name != "boundary":
            points = [(pt.x,-pt.y) for pt in region.getPoints()]
            poly = Polygon.Polygon(points)
            polyList.append(poly)

    bound_zero = Polygon.Polygon([(0,0),(1, 0), (1, -1), (0, -1)])
    bound_width = Polygon.Polygon([(width,0),(width-1, 0), (width-1, -1), (width, -1)])
    bound_height = Polygon.Polygon([(0,-height),(-1, -height), (-1, -height+1), (0, -height+1)])
    bound_all = Polygon.Polygon([(width,-height),(width-1, -height), (width-1, -height+1), (width, -height+1)])

    polyList.extend([bound_zero, bound_width, bound_height, bound_all])

    #use boundary size for image size
    Polygon.IO.writeSVG(fout, polyList, width=width,height=height, fill_color=("(255,255,255)",))
    fout_list.append(fout) #return the file name

    return fout_list

def svg2png(svgFile):
    svg = rsvg.Handle(file=svgFile)

    #This block converts the svg to png and applies naming conventions
    imgWidth=svg.props.width
    imgHeight=svg.props.height
    img = cairo.ImageSurface(cairo.FORMAT_ARGB32, imgWidth, imgHeight)
    ctx = cairo.Context(img)
    handler = rsvg.Handle(svgFile)
    handler.render_cairo(ctx)

    # flip x, y
    pat = cairo.SurfacePattern(img)

    # another way to flip an image on the Y axis
    # the order of the translate and scale are important
    m = cairo.Matrix()
    m.translate(0, img.get_height())
    m.scale(1, -1)

    pat.set_matrix(m)

    dest = cairo.ImageSurface(cairo.FORMAT_ARGB32, img.get_width(), img.get_height())
    cr = cairo.Context(dest)
    cr.set_source(pat)
    cr.paint()

    dest.write_to_png(svgFile.replace(".svg",".png"))

    #img.write_to_png(svgFile.replace(".svg",".png"))

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert region file to PNG file. Output to the same directory.")
    parser.add_argument('region_file', type=str, help='Path to region file')

    args, unknown = parser.parse_known_args()
    print args

    svgFile_list = region2svg(args.region_file, args.region_file.replace('.regions','.png'))

    for svgFile in svgFile_list:
        svg2png(svgFile)

