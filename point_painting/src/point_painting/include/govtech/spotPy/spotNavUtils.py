import google.protobuf.timestamp_pb2
import math
import numpy as np
import numpy.linalg
import os
import sys
import time

from bosdyn.api.graph_nav import map_pb2
from bosdyn.api import geometry_pb2
from bosdyn.client.frame_helpers import *
from bosdyn.client.math_helpers import *

def _create_edge_Xform(curr_wp_tform_to_wp, world_tform_curr_wp):
    # Concatenate the edge transform.
    world_tform_to_wp = np.dot(world_tform_curr_wp, curr_wp_tform_to_wp)
    return world_tform_to_wp

def _getWayPtWithName(current_graph, nameIn):
    for waypoint in current_graph.waypoints:
        if waypoint.annotations.name == nameIn:
            return waypoint
    print("Find WayPt Name ERROR: Could not find waypoint with name %s" %nameIn)
    return None

def _distance(a, b):
    return  math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

def _point_line_distance(point, start, end):
    if (start == end):
        return _distance(point, start)
    else:
        n = abs(
            (end[0] - start[0]) * (start[1] - point[1]) -
            (start[0] - point[0]) * (end[1] - start[1])
        )
        d = math.sqrt(
            (end[0] - start[0]) ** 2 + (end[1] - start[1]) ** 2
        )
        return n / d

def _rdp(wayPtPairList, epsilon):
    """ Ramer-Douglas-Peucker algorithm for line segment simplification
    """
    dmax = 0.0
    index = 0
    for i in range(1, len(wayPtPairList) - 1):
        d = _point_line_distance(wayPtPairList[i][1], wayPtPairList[0][1], wayPtPairList[-1][1])
        if d > dmax:
            index = i
            dmax = d

    if dmax >= epsilon:
        results = _rdp(wayPtPairList[:index+1], epsilon)[:-1] + _rdp(wayPtPairList[index:], epsilon)
    else:
        results = [wayPtPairList[0], wayPtPairList[-1]]

    return results

def computeWayPtPositions(current_graph, current_waypoints):
    # Perform a breadth first search of the graph starting from an arbitrary waypoint. Graph nav graphs
    # have no global reference frame. The only thing we can say about waypoints is that they have relative
    # transformations to their neighbors via edges.
    # This function returns the positions of the waypoints in the graph layout
    queue = []
    queue.append((current_graph.waypoints[0], np.eye(4)))
    visited = {}
    wayPointPosMap = {}

    # Breadth first search.
    while len(queue) > 0:
        # Visit a waypoint.
        curr_element = queue[0]
        queue.pop(0)
        curr_waypoint = curr_element[0]
        visited[curr_waypoint.id] = True

        # We now know the global pose of this waypoint, so set the pose.
        world_tform_current_waypoint = curr_element[1]
        wayPointPosMap[curr_waypoint.id] =  world_tform_current_waypoint[:3, 3]

        # Now, for each edge, walk along the edge and concatenate the transform to the neighbor.
        for edge in current_graph.edges:
            # If the edge is directed away from us...
            if edge.id.from_waypoint == curr_waypoint.id and edge.id.to_waypoint not in visited:
                current_waypoint_tform_to_waypoint = SE3Pose.from_obj(
                    edge.from_tform_to).to_matrix()
                world_tform_to_wp = _create_edge_Xform(current_waypoint_tform_to_waypoint,
                                                       world_tform_current_waypoint)
                # Add the neighbor to the queue.
                queue.append((current_waypoints[edge.id.to_waypoint], world_tform_to_wp))
                avg_pos += world_tform_to_wp[:3, 3]
            # If the edge is directed toward us...
            elif edge.id.to_waypoint == curr_waypoint.id and edge.id.from_waypoint not in visited:
                current_waypoint_tform_from_waypoint = (SE3Pose.from_obj(
                    edge.from_tform_to).inverse()).to_matrix()
                world_tform_from_wp = _create_edge_Xform(current_waypoint_tform_from_waypoint,
                                                         world_tform_current_waypoint)
                # Add the neighbor to the queue.
                queue.append((current_waypoints[edge.id.from_waypoint], world_tform_from_wp))
                avg_pos += world_tform_from_wp[:3, 3]

    return wayPointPosMap

def getOrderedWayPts(current_graph):
    retPts = []
    for i in range(0, len(current_graph.waypoints)):
        cName = "waypoint" + str(i)
        cPt = _getWayPtWithName(current_graph, cName)
        retPts.append(cPt)
    return retPts

def getOrderedWayPtsAndPositions(current_graph, current_waypoints, do_simplify, epsilon):
    retData = []
    allPts = getOrderedWayPts(current_graph)
    ptsPosMap = computeWayPtPositions(current_graph, current_waypoints)

    for cPt in allPts:
        retData.append(cPt, ptsPosMap[cPt.id])

    if do_simplify == False:
        return retData

    retData = _rdp(retData, epsilon)    
    return retData