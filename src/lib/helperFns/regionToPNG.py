import argparse
import re, Polygon, Polygon.IO
import sys, os
p = os.path.abspath(__file__)
sys.path.append(os.path.join(os.path.dirname(p), "../"))
import cairo, rsvg

import regions

#### USE REGION COMBINATION!
def region2svg(regionFile):
    """
    Converts region file to svg
    This is from the deprecated regions file with slight changes for
    proper calculation of the size of the regions map
    """
    fout= regionFile.replace(".regions", ".svg")
    rfi = regions.RegionFileInterface()
    rfi.readFile(regionFile)

    polyList = []

    for region in rfi.regions:
        if region.name != "boundary":
            points = [(pt.x,-pt.y) for pt in region.getPoints()]
            poly = Polygon.Polygon(points)
            polyList.append(poly)
        else:
            width=region.size.width
            height=region.size.height


    #use boundary size for image size
    Polygon.IO.writeSVG(fout, polyList,width=width,height=height, fill_color=("(255,255,255)",))   # works better than width=width,height=height
    #Polygon.IO.writeSVG(fout, polyList, fill_color=("(255,255,255)",))   # works better than width=width,height=height

    return fout #return the file name

def svg2png(svgFile):
    svg = rsvg.Handle(file=svgFile)

    #This block converts the svg to png and applies naming conventions
    imgWidth=svg.props.width
    imgHeight=svg.props.height
    img = cairo.ImageSurface(cairo.FORMAT_ARGB32, imgWidth, imgHeight)
    ctx = cairo.Context(img)
    handler = rsvg.Handle(svgFile)
    handler.render_cairo(ctx)
    img.write_to_png(svgFile.replace(".svg",".png"))

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert region file to PNG file. Output to the same directory.")
    parser.add_argument('region_file', type=str, help='Path to region file')

    args, unknown = parser.parse_known_args()
    print args

    svgFile = region2svg(args.region_file)
    svg2png(svgFile)