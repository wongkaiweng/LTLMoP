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


plotting = False
# scaling and offset for regionEditor.py
scale = 500  #100
offset_x = 500 # pixels #100
offset_y = 500 # pixels #100

def gen_world_regions(roads, output_file):
    road_regions= []
    lane_list = []
    map_polygon = Polygon.Polygon()
    for sindex in range(roads.number_of_segments()):
        segment = roads.get_mapped_segment(sindex)
        new_road_regions, new_road_lane, map_polygon = road_segment_regions((segment[0], segment[1]),
                                        (segment[2], segment[3]),
                                        prefix='segment_'+str(sindex)+'_',
                                        x1_intersection=roads.has_intersection_start(sindex),
                                        x2_intersection=roads.has_intersection_end(sindex),
                                        map_polygon=map_polygon)
        road_regions.extend(new_road_regions)
        lane_list.append(new_road_lane)

    # add boundary
    obstacles_and_boundary_regions = make_boundary_and_obstacles(road_regions)
    interface = regions.RegionFileInterface(regions = road_regions+obstacles_and_boundary_regions)

    # here we also want to remove the lanes in the middle
    interface.recalcAdjacency(lane_list)
    # also output lane_list to a file
    f_lane = open(output_file+'_transitions_to_exclude', 'w')
    f_lane.write(str(lane_list))
    f_lane.close()

    # build restrictions not to go backwards
    roadTransitions = road_restrictions(interface.transitions, interface.regions)
    ltl_transitions = []
    for origin, destList in roadTransitions.iteritems():
        ltl_transitions.append('[](e.'+origin+'_rc -> (s.'+origin+'|'+"|".join(["s."+x for x in destList])+'))')

    f_transitions = open(output_file.replace('.regions','.ltl'), 'w')
    f_transitions.write("&\n".join(ltl_transitions))
    f_transitions.close()

    interface.writeFile(output_file)
    return road_regions

def createRect(name, x, y, angle, height, length):
    """
    create Rectangle based on left top corner, angle and length
    height: y-direction
    length: x-direction
    """
    rect = regions.Region(name=name) #position=regions.Point(x, y)
    rect.addPoint(regions.Point(x,y),0) # origin
    rect.addPoint(regions.Point(x+height*sin(angle)-rect.position.x,y-height*cos(angle)-rect.position.y),1) # left bottom
    rect.addPoint(regions.Point(x+height*sin(angle)+length*cos(angle)-rect.position.x,\
                                y-height*cos(angle)+length*sin(angle)-rect.position.y),2) # right bottom
    rect.addPoint(regions.Point(x+length*cos(angle)-rect.position.x,y+length*sin(angle)-rect.position.y),3) # right top

    return rect

def round_floating_points(regionObjectList):
    """
    round off all floating points to make regions prettier.
    """
    for regionObject in regionObjectList:
        for point in regionObject.pointArray:
            point.x = round(point.x)
            point.y = round(point.y)
        regionObject.position.x = round(regionObject.position.x)
        regionObject.position.y = round(regionObject.position.y)
        regionObject.size.x = round(regionObject.size.x)
        regionObject.size.y = round(regionObject.size.y)


def road_segment_regions(x1, x2, prefix='straightroad',
                 x1_intersection=False, x2_intersection=False, map_polygon=None):
    assert len(x1) == 2 and len(x2) == 2


    # shift all regions to ensure it is shown in regionEditor
    x1 = (x1[0] + offset_x/scale, x1[1] + offset_y/scale)
    x2 = (x2[0] + offset_x/scale, x2[1] + offset_y/scale)     #x2 is bottom right corner. we are not using that.

    center = ((x1[0] + x2[0])/2.0, (x1[1] + x2[1])/2.0)
    length = math.sqrt((x1[0]-x2[0])**2 + (x1[1]-x2[1])**2) # length + 1.2 = with intersection as well
    angle = math.atan2(x2[1]-x1[1], x2[0]-x1[0])

    #-pi to pi
    if angle == math.pi/2:
        angle_prefix = "pi2_"
    else:
        angle_prefix = ""

    intersection_width = 0.6
    outputRegionList = []
    outputLane = None
                                    #shift from center                                        # shifting more
    angle_adjusted_x1_0 = center[0]-length/2*cos(angle) - intersection_width*sin(angle) - intersection_width*cos(angle)
    angle_adjusted_x1_1 = center[1]-length/2*sin(angle) + intersection_width*cos(angle) - intersection_width*sin(angle)

    # initialize regions
    region_x1_intersection = None
    region_x2_intersection = None
    region_lane_top = None
    region_lane_bottom = None

    ax1 = None
    if x1_intersection and x2_intersection:
        lane_marker_center = center
        lane_marker_length = length-1.2
        #print "lane_marker_length:" + str(lane_marker_length)
        #print "intersection_width:" + str(intersection_width)
        #print "length:" + str(length)

        # 4 regions in total
        # intersections
        region_x1_intersection = createRect(prefix+angle_prefix+'left_full_intersect', angle_adjusted_x1_0*scale, angle_adjusted_x1_1*scale, angle,\
                                             intersection_width*2*scale, intersection_width*2*scale)

        box_right_top_0 = angle_adjusted_x1_0 + length*cos(angle)
        box_right_top_1 = angle_adjusted_x1_1 + length*sin(angle)
        region_x2_intersection = createRect(prefix+angle_prefix+'right_full_intersect',box_right_top_0*scale, box_right_top_1*scale, angle,\
                                             intersection_width*2*scale, intersection_width*2*scale)

        # 2 lanes
        shiftedTop_x1_0 = angle_adjusted_x1_0+intersection_width*2*cos(angle)
        shiftedTop_x1_1 = angle_adjusted_x1_1+intersection_width*2*sin(angle)
        region_lane_top = createRect(prefix+angle_prefix+'top_lane',shiftedTop_x1_0*scale, shiftedTop_x1_1*scale, angle,\
                                             intersection_width*scale, lane_marker_length*scale)

        shiftedBottom_x1_0 = shiftedTop_x1_0+intersection_width*sin(angle) #height=1.2
        shiftedBottom_x1_1 = shiftedTop_x1_1-intersection_width*cos(angle)
        region_lane_bottom = createRect(prefix+angle_prefix+'bottom_lane', shiftedBottom_x1_0*scale, shiftedBottom_x1_1*scale, angle,\
                                             intersection_width*scale, lane_marker_length*scale)


    elif x1_intersection and (not x2_intersection):
        lane_marker_length = length-0.6#+0.06
        lane_marker_center = (x2[0] + (lane_marker_length/2.0 )*(x1[0]-x2[0])/length,
                              x2[1] + (lane_marker_length/2.0 )*(x1[1]-x2[1])/length) #- 0.06

        # 4 regions in total (right intersect smaller)
        # intersections
        region_x1_intersection = createRect(prefix+angle_prefix+'left_full_intersect', angle_adjusted_x1_0*scale, angle_adjusted_x1_1*scale, angle,\
                                             intersection_width*2*scale, intersection_width*2*scale)

        box_right_top_0 = angle_adjusted_x1_0 + length*cos(angle) + intersection_width*cos(angle)#+ 0.06*cos(angle)
        box_right_top_1 = angle_adjusted_x1_1 + length*sin(angle) + intersection_width*sin(angle)#+ 0.06*sin(angle)
        region_x2_intersection = createRect(prefix+angle_prefix+'right_intersect',box_right_top_0*scale, box_right_top_1*scale, angle,\
                                             intersection_width*2*scale, (intersection_width)*scale) #-0.06

        # 2 lanes
        shiftedTop_x1_0 = angle_adjusted_x1_0+intersection_width*2*cos(angle)
        shiftedTop_x1_1 = angle_adjusted_x1_1+intersection_width*2*sin(angle)
        region_lane_top = createRect(prefix+angle_prefix+'top_lane',shiftedTop_x1_0*scale, shiftedTop_x1_1*scale, angle,\
                                             intersection_width*scale, lane_marker_length*scale)

        shiftedBottom_x1_0 = shiftedTop_x1_0+intersection_width*sin(angle) #height=1.2
        shiftedBottom_x1_1 = shiftedTop_x1_1-intersection_width*cos(angle)
        region_lane_bottom = createRect(prefix+angle_prefix+'bottom_lane', shiftedBottom_x1_0*scale, shiftedBottom_x1_1*scale, angle,\
                                             intersection_width*scale, lane_marker_length*scale)


    elif (not x1_intersection) and x2_intersection:
        lane_marker_length = length-0.6#+0.06
        lane_marker_center = (x1[0] + (lane_marker_length/2.0 )*(x2[0]-x1[0])/length,
                              x1[1] + (lane_marker_length/2.0 )*(x2[1]-x1[1])/length) #- 0.06

        # 4 regions in total (left intersect smaller)
        # intersections
        region_x1_intersection = createRect(prefix+angle_prefix+'left_intersect', angle_adjusted_x1_0*scale, angle_adjusted_x1_1*scale, angle,\
                                             intersection_width*2*scale, (intersection_width)*scale) #-0.06

        box_right_top_0 = angle_adjusted_x1_0 + length*cos(angle)
        box_right_top_1 = angle_adjusted_x1_1 + length*sin(angle)
        region_x2_intersection = createRect(prefix+angle_prefix+'right_full_intersect',box_right_top_0*scale, box_right_top_1*scale, angle,\
                                             intersection_width*2*scale, intersection_width*2*scale)

        # 2 lanes
        shiftedTop_x1_0 = angle_adjusted_x1_0+(intersection_width)*cos(angle) #-0.06
        shiftedTop_x1_1 = angle_adjusted_x1_1+(intersection_width)*sin(angle) #-0.06
        region_lane_top = createRect(prefix+angle_prefix+'top_lane',shiftedTop_x1_0*scale, shiftedTop_x1_1*scale, angle,\
                                             intersection_width*scale, lane_marker_length*scale)

        shiftedBottom_x1_0 = shiftedTop_x1_0+intersection_width*sin(angle) #height=1.2
        shiftedBottom_x1_1 = shiftedTop_x1_1-intersection_width*cos(angle)
        region_lane_bottom = createRect(prefix+angle_prefix+'bottom_lane', shiftedBottom_x1_0*scale, shiftedBottom_x1_1*scale, angle,\
                                             intersection_width*scale, lane_marker_length*scale)

    else:
        lane_marker_center = center
        lane_marker_length = length #+0.12

        # 4 regions in total (short intersection)
        # intersections
        region_x1_intersection = createRect(prefix+angle_prefix+'left_intersect', angle_adjusted_x1_0*scale, angle_adjusted_x1_1*scale, angle,\
                                             intersection_width*2*scale, intersection_width*scale)

        box_right_top_0 = angle_adjusted_x1_0 + length*cos(angle) + intersection_width*cos(angle)
        box_right_top_1 = angle_adjusted_x1_1 + length*sin(angle) + intersection_width*sin(angle)
        region_x2_intersection = createRect(prefix+angle_prefix+'right_intersect',box_right_top_0*scale, box_right_top_1*scale, angle,\
                                             intersection_width*2*scale, intersection_width*scale)

        # 2 lanes
        shiftedTop_x1_0 = angle_adjusted_x1_0+intersection_width*cos(angle)
        shiftedTop_x1_1 = angle_adjusted_x1_1+intersection_width*sin(angle)
        region_lane_top = createRect(prefix+angle_prefix+'top_lane',shiftedTop_x1_0*scale, shiftedTop_x1_1*scale, angle,\
                                             intersection_width*scale, (lane_marker_length)*scale)

        shiftedBottom_x1_0 = shiftedTop_x1_0+intersection_width*sin(angle) #height=1.2
        shiftedBottom_x1_1 = shiftedTop_x1_1-intersection_width*cos(angle)
        region_lane_bottom = createRect(prefix+angle_prefix+'bottom_lane', shiftedBottom_x1_0*scale, shiftedBottom_x1_1*scale, angle,\
                                             intersection_width*scale, (lane_marker_length)*scale)

    # do some round off of floating points
    round_floating_points([region_x1_intersection,region_x2_intersection,region_lane_top,region_lane_bottom])

    # check if region already exists with polygon. If existed, then we don't add this region
    for regionObject in [region_x1_intersection,region_x2_intersection,region_lane_top,region_lane_bottom]:
        regionPolygon = Polygon.Polygon([(pt.x,pt.y) for pt in regionObject.getPoints()])
        # if new region does not overlap with curret map, then add to region list and union polygon
        # ***cannot remove corner ones, but removed intersections
        if not map_polygon or not map_polygon.covers(regionPolygon):

            # add another check for overlapping
            if map_polygon.overlaps(regionPolygon):
                # subtract extra with map and then add it in
                regionPolygon = regionPolygon - map_polygon
                region_modified = regions.Region(name=regionObject.name) #position=regions.Point(x, y)

                if not len(regionPolygon) == 1:
                    print "!!!! more than one contours from substraction of region with map"

                for idx, pt in enumerate(regionPolygon[0]):
                    region_modified.addPoint(regions.Point(pt[0]-region_modified.position.x, pt[1]-region_modified.position.y),idx)

                round_floating_points([region_modified])
                map_polygon |= regionPolygon
                outputRegionList.append(region_modified)
            else:
                map_polygon |= regionPolygon
                outputRegionList.append(regionObject)
        #else:
        #    print "not adding region!"

    #outputRegionList.extend([region_x1_intersection,region_x2_intersection,region_lane_top,region_lane_bottom])

    # now also export lanes in the middle
    outputLane = [[round(shiftedBottom_x1_0*scale), round(shiftedBottom_x1_1*scale)],\
                  [round(shiftedBottom_x1_0+lane_marker_length*cos(angle))*scale,\
                   round(shiftedBottom_x1_1+lane_marker_length*sin(angle))*scale]]

    # plot debugging
    if plotting:
        fig1 = plt.figure()
        ax1 = fig1.add_subplot(111, aspect='equal')
        for region in outputRegionList:
            ax1.plot([rect.x for rect in region.getPoints()]+[list(region.getPoints())[0].x],\
                     [rect.y for rect in region.getPoints()]+[list(region.getPoints())[0].y])

        ax1.add_patch(patches.RegularPolygon((center[0]*scale, center[1]*scale), 4,\
                                             radius=math.sqrt((length+1.2)**2+(length+1.2)**2)*scale/2,\
                                                orientation=math.pi/4+angle))
        ax1.add_patch(patches.RegularPolygon((lane_marker_center[0]*scale, lane_marker_center[1]*scale), 4,\
                                             radius=math.sqrt((lane_marker_length)**2+(lane_marker_length)**2)*scale/2,\
                                                orientation=math.pi/4+angle))
        ax1.plot(center[0]*scale, center[1]*scale,'*')
        ax1.plot(lane_marker_center[0]*scale, lane_marker_center[1]*scale,'o')
        ax1.plot(angle_adjusted_x1_0*scale,angle_adjusted_x1_1*scale,'*')
        ax1.axis('equal')

        plt.show()

    return outputRegionList, outputLane, map_polygon

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


def road_restrictions(transitions, regions):
    """
    build restrictions (since each lane can only go one way)
    """
    roadTransition = {} # store real network info

    for OriginNo in range(len(regions)):
        # skip boundaries and obstacles
        if regions[OriginNo].name == 'boundary' or regions[OriginNo].isObstacle:
            continue

        # find origin center
        origin_regionPolygon = Polygon.Polygon([(pt.x,pt.y) for pt in regions[OriginNo].getPoints()])
        origin_center = origin_regionPolygon.center(0)

        for DestNo in range(len(regions)):
            # skip boundaries and obstacles
            if regions[DestNo].name == 'boundary' or regions[DestNo].isObstacle:
                continue

            # find origin center
            dest_regionPolygon = Polygon.Polygon([(pt.x,pt.y) for pt in regions[DestNo].getPoints()])
            dest_center = dest_regionPolygon.center(0)

            if transitions[OriginNo][DestNo]:
                # check where that face is on original region
                # from center check face center angle
                for face in transitions[OriginNo][DestNo]:
                    face_pt0,face_pt1 = face
                    face_center = ((face_pt0[0] + face_pt1[0])/2.0, (face_pt0[1] + face_pt1[1])/2.0)

                    # calculate angle from face to two regions
                    angle_origin = math.atan2(face_center[1]-origin_center[1], face_center[0]-origin_center[0])
                    angle_dest = math.atan2(face_center[1]-dest_center[1], face_center[0]-dest_center[0])

                    # check if region is rotated
                    if "pi2" in regions[OriginNo].name:
                        rotate_ninety_origin = True
                    else:
                        rotate_ninety_origin = False

                    if "pi2" in regions[DestNo].name:
                        rotate_ninety_dest = True
                    else:
                        rotate_ninety_dest = False

                    #print '==================================================='
                    #print "face:" + str(face)
                    #print '---------------------------------------------------'
                    origin_direction = findDirection(regions, OriginNo, angle_origin, rotate_ninety_origin, True)
                    #print "Origin: " + regions[OriginNo].name + " - angle: " + str(angle_origin) + " -direction:" + str(origin_direction)

                    #print '---------------------------------------------------'
                    dest_direction = findDirection(regions, DestNo, angle_dest, rotate_ninety_dest, False)
                    #print "Dest: " + regions[DestNo].name + " - angle: " + str(angle_dest) + " -direction:" + str(dest_direction)

                    # check where that face is on destination region
                    if numpy.dot(origin_direction, dest_direction) > 0:
                        # same direction
                        # print "Origin:"+regions[OriginNo].name+' and '+regions[DestNo].name +' are connected!'
                        if regions[OriginNo].name not in roadTransition.keys():
                            roadTransition[regions[OriginNo].name] = []

                        if regions[DestNo].name not in roadTransition[regions[OriginNo].name]:
                            roadTransition[regions[OriginNo].name].append(regions[DestNo].name)

    #print roadTransition
    return roadTransition

def findDirection(regions, idx, faceAngle, rotate_ninety, isOrigin):
    """
    find direction of the face
    origin and dest are different (origin always go out)
    """

    if "full_intersect" in regions[idx].name:
        # depends on the angle
        if faceAngle < 0:
            faceAngle += 2*math.pi

        if isOrigin:
            # origin exiting version
            possible_directions = [[1,0],[0,1],[0,1],[-1,0],\
                                    [-1,0],[0,-1],[0,-1],[1,0]]
        else:
            # dest arriving version
            #possible_directions = [[1,0],[0,-1],[0,1],[1,0],\
            #                       [-1,0],[0,1],[0,-1],[-1,0]]
            possible_directions = [[-1,0],[0,-1],[0,1],[-1,0],\
                                   [1,0],[0,1],[0,-1],[1,0]]
            #print 'dest-direction:' + str(possible_directions[int(faceAngle/(math.pi/4))])
        direction = possible_directions[int(faceAngle/(math.pi/4))]

        return direction

    else:
        if "top" in regions[idx].name:
            direction = [-1,0]
            #direction = [1,0]
        elif "bottom" in regions[idx].name:
            #direction = [-1,0]
            direction = [1,0]
        elif "left_intersect" in regions[idx].name:
            #direction = [0,1]
            direction = [0,-1]
        elif "right_intersect" in regions[idx].name:
            #direction = [0,-1]
            direction = [0,1]
        else:
            print "does not match key word"

        #print "direction:" + str(direction)
        if rotate_ninety:
            direction = rotation_matrix(direction, math.pi/2)
            #print "rotated 90"

        # based on face, find angle vector
        angle_vector = [math.cos(faceAngle), math.sin(faceAngle)]
        #print "faceAngle:" + str(faceAngle) + ", angle_vector:"  +str(angle_vector)

        if not isOrigin:
            #print "not origin"
            if numpy.dot(angle_vector, direction) <0:
                angle_vector = [i * -1 for i in angle_vector]

        return angle_vector

def rotation_matrix(axis, theta):
    rotMatrix = numpy.array([[numpy.cos(theta), -numpy.sin(theta)],
                             [numpy.sin(theta),  numpy.cos(theta)]])
    return numpy.dot(rotMatrix, axis)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Supply .json file with rnd dict. e.g:\n\
    {\n\
    'version': 0,\n\
    'length': 3,\n\
    'transform': [0, 0, 0],\n\
    \n\
    '_comment': 'no of roads segments',\n\
    'shape': [3, 4]\n\
    }")
    parser.add_argument('FILE', type=str, help='road network description file')
    parser.add_argument('output_region_file_dir', type=str, help='output region file name with directory,e.g.:output.regions')

    args = parser.parse_args()

    with open(args.FILE, 'rt') as f:
        roads = RoadNetwork(f)

    gen_world_regions(roads, args.output_region_file_dir)

    #Desktop/fmrbenchmark/domains/dubins_traffic/dubins_traffic_utils/examples/trialsconf/mc-small-4grid-agents2.json
    print "Region Generation DONE!"

