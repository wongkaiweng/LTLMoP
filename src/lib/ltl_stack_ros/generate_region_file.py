# get robot states
#rostopic echo /gazebo/model_states
# msgtype:gazebo_msgs/ModelStates


# RoadNetwork
#Desktop/fmrbenchmark/tools/fmrb-pkg/fmrb/dubins_traffic.py

# sdf file with road segments
#catherine@catherine-ThinkPad-X1:~/dubsim_workspace/install/lib/dub_sim$ python genroad.py  ../../../mydata.json
#{"shape": [3, 4], "_comment": "no of roads segments", "version": 0, "length": 3, "transform": [0, 0, 0]}

"""
        "points": offset from top left corner
        "position": top left corner
        "type": "poly",
        "size": -boundingbox
"""
import os, sys
import argparse
from fmrb.dubins_traffic import RoadNetwork
import math
from math import sin, cos
import Polygon
import numpy

import matplotlib.pyplot as plt
import matplotlib.patches as patches

p = os.path.abspath(__file__)
#print os.path.join(os.path.dirname(p), "../")
sys.path.append(os.path.join(os.path.dirname(p), "../"))
import regions


plotting = True
# scaling and offset for regionEditor.py
scale = 0.20  #100
offset_x = 820 # pixels #100
offset_y = 1250 # pixels #100

region_dict = {'left_lane':[(482, -1237),(2100, -1240), (2822,-855), (3272, 68),(2250, 49),\
                            (2070, -202), (515, -227), (250, 1.65), (-775.8, -5.6), (-467, -785)],
               'top_lane':[(-775.8, -5.6), (250, 1.65), (235, 1744), (213, 3508), (-810, 3507)],
               'right_lane':[(-810, 3507), (213, 3508), (444, 3772), (2009, 3767), (2188, 3543), \
                             (3237, 3529), (2872, 4442), (1910, 4800), (475, 4785), (-385, 4454)],
               'bottom_lane':[(2250, 49), (3272, 68), (3237, 3529), (2188, 3543), (2225, 1783)],
               'left_ground':[(515, -227), (2070, -202), (2250, 49), (2225, 1783), (235, 1744),\
                              (250, 1.65)],
               'right_ground':[(235, 1744), (2225, 1783), (2188, 3543), (2009, 3767), (444, 3772),\
                               (213, 3508)]}

# faces to remove
to_remove_faces = [[[2009, 3767],[444, 3772]]]


def gen_world_regions(region_dict, to_remove_faces, output_file):

    region_obj_list= []
    map_polygon = Polygon.Polygon()

    region_obj_list, map_polygon = create_regions(region_dict, map_polygon=map_polygon)

    # add boundary
    obstacles_and_boundary_regions = make_boundary_and_obstacles(region_obj_list)
    interface = regions.RegionFileInterface(regions = region_obj_list+obstacles_and_boundary_regions)

    # here we also want to remove the lanes in the middle
    #offset_faces = get_offset_faces(to_remove_faces)
    #interface.recalcAdjacency(offset_faces)
    interface.recalcAdjacency()

    interface.writeFile(output_file)
    return region_obj_list

def get_offset_faces(to_remove_faces):
    offset_faces = []
    for [[x1, y1], [x2, y2]] in to_remove_faces:
        offset_faces.append([[(offset_x+x1)*scale, (offset_y+y1)*scale], \
                             [(offset_x+x2)*scale, (offset_y+y2)*scale]])

    print offset_faces
    return offset_faces

def createPoly(name, region_point_list):
    """
    create poly
    """
    x_origin, y_origin = 0,0
    poly = regions.Region(name=name) #position=regions.Point(x, y)
    for idx, (x, y) in enumerate(region_point_list):
        if not idx:
            x_origin, y_origin = x, y
        poly.addPoint(regions.Point(((offset_x+x)*scale-poly.position[0]),\
                                    ((offset_y+y)*scale-poly.position[1])),idx)

    return poly

def round_floating_points(regionObjectList):
    """
    round off all floating points to make regions prettier.
    """
    for regionObject in regionObjectList:
        for point in regionObject.pointArray:
            point.x = int(round(point.x))
            point.y = int(round(point.y))
        regionObject.position.x = int(round(regionObject.position.x))
        regionObject.position.y = int(round(regionObject.position.y))
        regionObject.size.x = int(round(regionObject.size.x))
        regionObject.size.y = int(round(regionObject.size.y))


def create_regions(region_dict, map_polygon=None):

    outputRegionList = []

    ax1 = None

    for name, region_point_list in region_dict.iteritems():
        region_obj = createPoly(name, region_point_list)

        # do some round off of floating points
        round_floating_points([region_obj])

        regionPolygon = Polygon.Polygon([(pt.x,pt.y) for pt in region_obj.getPoints()])
        # if new region does not overlap with curret map, then add to region list and union polygon
        # ***cannot remove corner ones, but removed intersections
        # if not map_polygon or not map_polygon.covers(regionPolygon):

        #     # add another check for overlapping
        #     if map_polygon.overlaps(regionPolygon):
        #         # subtract extra with map and then add it in
        #         regionPolygon = regionPolygon - map_polygon
        #         region_modified = regions.Region(name=region_obj.name) #position=regions.Point(x, y)

        #         if not len(regionPolygon) == 1:
        #             print "!!!! more than one contours from substraction of region with map"

        #         for idx, pt in enumerate(regionPolygon[0]):
        #             region_modified.addPoint(regions.Point(pt[0]-region_modified.position.x, pt[1]-region_modified.position.y),idx)

        #         round_floating_points([region_modified])
        #         map_polygon |= regionPolygon
        #         outputRegionList.append(region_modified)
        #     else:
        map_polygon |= regionPolygon
        outputRegionList.append(region_obj)
        #else:
        #    print "not adding region!"

    #outputRegionList.extend([region_x1_intersection,region_x2_intersection,region_lane_top,region_lane_bottom])

    # plot debugging
    if plotting:
        fig1 = plt.figure()
        ax1 = fig1.add_subplot(111, aspect='equal')
        for region in outputRegionList:
            ax1.plot([rect.x for rect in region.getPoints()]+[list(region.getPoints())[0].x],\
                     [rect.y for rect in region.getPoints()]+[list(region.getPoints())[0].y])
            #ax1.plot([pt.x for pt in region.getPoints()],\
            #         [pt.y for pt in region.getPoints()])
            print [pt.x for pt in region.getPoints()], [pt.y for pt in region.getPoints()]

        #ax1.add_patch(patches.RegularPolygon((center[0]*scale, center[1]*scale), 4,\
        #                                     radius=math.sqrt((length+1.2)**2+(length+1.2)**2)*scale/2,\
        #                                        orientation=math.pi/4+angle))
        #ax1.add_patch(patches.RegularPolygon((lane_marker_center[0]*scale, lane_marker_center[1]*scale), 4,\
        #                                     radius=math.sqrt((lane_marker_length)**2+(lane_marker_length)**2)*scale/2,\
        #                                        orientation=math.pi/4+angle))
        #ax1.plot(center[0]*scale, center[1]*scale,'*')
        #ax1.plot(lane_marker_center[0]*scale, lane_marker_center[1]*scale,'o')
        #ax1.plot(angle_adjusted_x1_0*scale,angle_adjusted_x1_1*scale,'*')
        ax1.axis('equal')

        plt.show()

    return outputRegionList, map_polygon

def make_boundary_and_obstacles(regionsList):
    """
    create obstacles and boundary based on all roads
    """

    bound_poly = Polygon.Polygon()
    for r in regionsList:
        points = [(pt.x,pt.y) for pt in r.getPoints()]
        bound_poly += Polygon.Polygon(points)

    print '------------------------------------'
    print "Boundary Polygon:" + str(bound_poly)
    print '------------------------------------'

    # form holes regions
    obstacle_regions_list = []
    for idx_contour, contour in enumerate(bound_poly):
        if bound_poly.isHole(idx_contour):
            #print contour
            obstacle_region = regions.Region(name="hole" + str(idx_contour)) #position=regions.Point(x, y)
            for idx_pt, pt in enumerate(contour):
                obstacle_region.addPoint(regions.Point(pt[0]-obstacle_region.position.x, pt[1]-obstacle_region.position.y),idx_pt)
            obstacle_region.isObstacle = True

            obstacle_regions_list.append(obstacle_region)

        else: #this is boundary
            bound_region = regions.Region(name="boundary") #position=regions.Point(x, y)
            for idx_boundary, pt in enumerate(contour):
                bound_region.addPoint(regions.Point(pt[0]-bound_region.position.x, pt[1]-bound_region.position.y),idx_boundary)

    #for x in [region.getPoints() for region in obstacle_regions_list]:
    #    print list(x)
    #print [x for x in bound_region.getPoints()]

    round_floating_points(obstacle_regions_list + [bound_region])
    return obstacle_regions_list + [bound_region]



def rotation_matrix(axis, theta):
    rotMatrix = numpy.array([[numpy.cos(theta), -numpy.sin(theta)],
                             [numpy.sin(theta),  numpy.cos(theta)]])
    return numpy.dot(rotMatrix, axis)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Generate .json file for ros")

    parser.add_argument('output_region_file_dir', type=str, help='output region file name with directory,e.g.:output.regions')

    args = parser.parse_args()

    gen_world_regions(region_dict, to_remove_faces, args.output_region_file_dir)

    #Desktop/fmrbenchmark/domains/dubins_traffic/dubins_traffic_utils/examples/trialsconf/mc-small-4grid-agents2.json
    print "Region Generation DONE!"
