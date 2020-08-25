import argparse
import grpc
import logging
import math
import os
import sys
import time
import threading

from bosdyn.api import geometry_pb2
from bosdyn.api import power_pb2
from bosdyn.api import robot_state_pb2
from bosdyn.api.graph_nav import graph_nav_pb2
from bosdyn.api.graph_nav import map_pb2
from bosdyn.api.graph_nav import nav_pb2
import bosdyn.client.channel
from bosdyn.client.power import safe_power_off, PowerClient, power_on
from bosdyn.client.graph_nav import GraphNavClient
from bosdyn.client.frame_helpers import get_odom_tform_body
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive, LeaseWallet
from bosdyn.client.robot_command import RobotCommandClient, RobotCommandBuilder
from bosdyn.client.robot_state import RobotStateClient
import bosdyn.client.util
import google.protobuf.timestamp_pb2

SPOT_NAV_RUN_PTS_MODE = 0
SPOT_NAV_RUN_EDGE_MODE = 1

class SpotNavRun(object):
    """ Runs a recorded waypoint recording using Spot's Semi-Autonomy API"""
    def __init__(self, robot, robot_state_client, lease_client, lease_keepalive, graphFolder, graphBaseFolderName, run_mode = SPOT_NAV_RUN_PTS_MODE):
        self._graphFolder = graphFolder
        self._graphBaseFolderName = graphBaseFolderName
        self._nav_run_mode = run_mode
        self._robotLogFN = None

        self._robot = robot
        self._robot_state_client = robot_state_client
        self._lease_client = lease_client
        self._lease_wallet = self._lease_client.lease_wallet
        self._lease_keepalive = lease_keepalive

        # Create the client for the Graph Nav main service.
        self._graph_nav_client = self._robot.ensure_client(GraphNavClient.default_service_name)

        # Store the most recent knowledge of the state of the robot based on rpc calls.
        self._current_graph = None
        self._current_edges = dict()  #maps to_waypoint to list(from_waypoint)
        self._current_waypoint_snapshots = dict()  # maps id to waypoint snapshot
        self._current_edge_snapshots = dict()  # maps id to edge snapshot
        
        self._route = None
        self._route_is_finished = True
        self._routeTime = self._getTimeMS()
        self._routeNavIdx = 0
        self._routeNavWayPts = []

    def setRobotLogFN(self, fnIn):
        self._robotLogFN = fnIn

    def runRobotLog(self, msgIn):
        print(msgIn)
        if self._robotLogFN:
            self._robotLogFN(msgIn)          

    def _getTimeMS(self):
        return int(round(time.time() * 1000))

    def _id_to_short_code(self, id):
        """Convert a unique id to a 2 letter short code."""
        tokens = id.split('-')
        if len(tokens) > 2:
            return '%c%c' % (tokens[0][0], tokens[1][0])
        return None

    def _short_code_to_id(self, short_code):
        """Convert a 2 letter short code to a unique id."""
        if len(short_code) != 2:
            return short_code  # Not a short code.

        if self._current_graph is None:
            graph = self._graph_nav_client.download_graph()
            if graph is not None:
                self._current_graph = graph

        ret = short_code
        for waypoint in self._current_graph.waypoints:
            if short_code == self._id_to_short_code(waypoint.id):
                if ret != short_code:
                    return short_code  # Multiple waypoints with same short code.
                ret = waypoint.id
        return ret        

    def _get_localization_state(self, *args):
        """Get the current localization and state of the robot."""
        state = self._graph_nav_client.get_localization_state()
        print('Got localization: \n%s' % str(state.localization))
        odom_tform_body = get_odom_tform_body(state.robot_kinematics.transforms_snapshot)
        print('Got robot state in odom frame: \n%s' % str(odom_tform_body))

    def _set_initial_localization_fiducial(self, *args):
        """Trigger localization when near a fiducial."""
        robot_state = self._robot_state_client.get_robot_state()
        current_odom_tform_body = get_odom_tform_body(
            robot_state.kinematic_state.transforms_snapshot).to_proto()
        # Create an empty instance for initial localization since we are asking it to localize
        # based on the nearest fiducial.
        localization = nav_pb2.Localization()
        self._graph_nav_client.set_localization(initial_guess_localization=localization,
                                                ko_tform_body=current_odom_tform_body)

    def _set_initial_localization_waypoint(self, wayPtID):
        """Trigger localization to a waypoint given a short code"""
        destination_waypoint = wayPtID #self._short_code_to_id(sCode)

        robot_state = self._robot_state_client.get_robot_state()
        current_odom_tform_body = get_odom_tform_body(
            robot_state.kinematic_state.transforms_snapshot).to_proto()
        # Create an initial localization to the specified waypoint as the identity.
        localization = nav_pb2.Localization()
        localization.waypoint_id = destination_waypoint
        localization.waypoint_tform_body.rotation.w = 1.0
        self._graph_nav_client.set_localization(
            initial_guess_localization=localization,
            # It's hard to get the pose perfect, search +/-20 deg and +/-20cm (0.2m).
            max_distance = 0.2,
            max_yaw = 20.0 * math.pi / 180.0,
            fiducial_init=graph_nav_pb2.SetLocalizationRequest.FIDUCIAL_INIT_NO_FIDUCIAL,
            ko_tform_body=current_odom_tform_body)
        print("Localizing to waypoint: %s" %wayPtID)

    def _getGraphDataFilename(self, idx, extStr):
        filename = self._graphBaseFolderName + "/navGraph_" + str(idx) + "." + extStr
        print("_getGraphDataFilename: %s" %filename)
        return filename

    def _upload_graph_and_snapshots(self):
        """Upload the graph and snapshots to the robot."""
        self.runRobotLog("Loading the graph from disk into local storage...")
        with open(self._getGraphDataFilename(0, "spotGraph"), "rb") as graph_file:
            # Load the graph from disk.
            data = graph_file.read()
            self._current_graph = map_pb2.Graph()
            self._current_graph.ParseFromString(data)
            self.runRobotLog("Loaded graph has {} waypoints and {} edges".format(
                len(self._current_graph.waypoints), len(self._current_graph.edges)))

        cnt = 0
        for waypoint in self._current_graph.waypoints:
            # Load the waypoint snapshots from disk.
            with open(self._getGraphDataFilename(cnt, "spotWaypts"),
                      "rb") as snapshot_file:
                waypoint_snapshot = map_pb2.WaypointSnapshot()
                waypoint_snapshot.ParseFromString(snapshot_file.read())
                self._current_waypoint_snapshots[waypoint_snapshot.id] = waypoint_snapshot
                cnt += 1

        cnt = 0
        for edge in self._current_graph.edges:
            # Load the edge snapshots from disk.
            with open(self._getGraphDataFilename(cnt, "spotEdges"),
                      "rb") as snapshot_file:
                edge_snapshot = map_pb2.EdgeSnapshot()
                edge_snapshot.ParseFromString(snapshot_file.read())
                self._current_edge_snapshots[edge_snapshot.id] = edge_snapshot
                cnt += 1

        # Upload the graph to the robot.
        self.runRobotLog("Uploading the graph and snapshots to the robot...")
        self._lease = self._lease_wallet.get_lease()
        self._graph_nav_client.upload_graph(lease=self._lease.lease_proto,
                                            graph=self._current_graph)

        # Upload the snapshots to the robot.
        for waypoint_snapshot in self._current_waypoint_snapshots.values():
            self._graph_nav_client.upload_waypoint_snapshot(waypoint_snapshot)
            self.runRobotLog("Uploaded {}".format(waypoint_snapshot.id))
        for edge_snapshot in self._current_edge_snapshots.values():
            self._graph_nav_client.upload_edge_snapshot(edge_snapshot)
            self.runRobotLog("Uploaded {}".format(edge_snapshot.id))

        # The upload is complete! Check that the robot is localized to the graph,
        # and it if is not, prompt the user to localize the robot before attempting
        # any navigation commands.
        localization_state = self._graph_nav_client.get_localization_state()
        if not localization_state.localization.waypoint_id:
            # The robot is not localized to the newly uploaded graph.
            print("\n")
            print("Upload complete! The robot is currently not localized to the map; please localize")

    def _check_success(self, command_id=-1):
        """Use a navigation command id to get feedback from the robot and sit when command succeeds."""
        if command_id == -1:
            # No command, so we have not status to check.
            return False
        status = self._graph_nav_client.navigation_feedback(command_id)
        if status.status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_REACHED_GOAL:
            # Successfully completed the navigation commands!
            return True
        elif status.status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_LOST:
            self.runRobotLog("Robot got lost when navigating the route, the robot will now sit down.")
            return True
        elif status.status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_STUCK:
            self.runRobotLog("Robot got stuck when navigating the route, the robot will now sit down.")
            return True
        elif status.status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_ROBOT_IMPAIRED:
            self.runRobotLog("Robot is impaired.")
            return True
        else:
            # Navigation command is not complete yet.
            return False

    def _match_edge(self, current_edges, waypoint1, waypoint2):
        """Find an edge in the graph that is between two waypoint ids."""
        # Return the correct edge id as soon as it's found.
        for edge_to_id in current_edges:
            for edge_from_id in current_edges[edge_to_id]:
                if (waypoint1 == edge_to_id) and (waypoint2 == edge_from_id):
                    # This edge matches the pair of waypoints! Add it the edge list and continue.
                    return map_pb2.Edge.Id(from_waypoint=waypoint2, to_waypoint=waypoint1)
                elif (waypoint2 == edge_to_id) and (waypoint1 == edge_from_id):
                    # This edge matches the pair of waypoints! Add it the edge list and continue.
                    return map_pb2.Edge.Id(from_waypoint=waypoint1, to_waypoint=waypoint2)
        return None

    def _getWayPtWithID(self, idIn):
        for waypoint in self._current_graph.waypoints:
            if waypoint.id == idIn:
                return waypoint
        return None

    def _getWayPtWithName(self, nameIn):
        for waypoint in self._current_graph.waypoints:
            if waypoint.annotations.name == nameIn:
                return waypoint
        self.runRobotLog("Find WayPt Name ERROR: Could not find waypoint with name %s" %nameIn)
        return None

    def _list_graph_waypoint_and_edge_ids(self, *args):
        """List the waypoint ids and edge ids of the graph currently on the robot."""
        graph = self._graph_nav_client.download_graph()
        localization_id = self._graph_nav_client.get_localization_state().localization.waypoint_id
        if graph is not None:
            self._current_graph = graph

            # Determine how many waypoints have the same short code.
            short_code_to_count = {}
            for waypoint in graph.waypoints:
                short_code = self._id_to_short_code(waypoint.id)
                if short_code not in short_code_to_count:
                    short_code_to_count[short_code] = 0
                short_code_to_count[short_code] += 1

            print('%d waypoints:' % len(graph.waypoints))
            for waypoint in graph.waypoints:
                short_code = self._id_to_short_code(waypoint.id)
                if short_code is None or short_code_to_count[short_code] != 1:
                    short_code = '  '  # If the short code is not valid/unique, don't show it.

                print("%s %s Waypoint id: %s name: %s" %
                      ('->' if localization_id == waypoint.id else '  ', short_code, waypoint.id,
                       waypoint.annotations.name))

            for edge in graph.edges:
                if edge.id.to_waypoint in self._current_edges:
                    if edge.id.from_waypoint not in self._current_edges[edge.id.to_waypoint]:
                        self._current_edges[edge.id.to_waypoint].append(edge.id.from_waypoint)
                else:
                    self._current_edges[edge.id.to_waypoint] = [edge.id.from_waypoint]
                
                fromID = edge.id.from_waypoint
                toID = edge.id.to_waypoint
                fromName = self._getWayPtWithID(fromID).annotations.name
                toName = self._getWayPtWithID(toID).annotations.name
                print("(Edge) from waypoint id: ", edge.id.from_waypoint, "[", fromName, "] and to waypoint id: ",
                      edge.id.to_waypoint, "[", toName, "]")                                                   

    def _navigate_to(self, *args):
        """Navigate to a specific waypoint."""
        # Take the first argument as the destination waypoint.
        if len(args) < 1:
            # If no waypoint id is given as input, then return without requesting navigation.
            self.runRobotLog("No waypoint provided as a destination for navigate to.")
            return

        self._lease = self._lease_wallet.get_lease()
        destination_waypoint = self._short_code_to_id(args[0][0])

        # Stop the lease keepalive and create a new sublease for graph nav.
        self._lease = self._lease_wallet.advance()
        sublease = self._lease.create_sublease()
        self._lease_keepalive.shutdown()

        # Navigate to the destination waypoint.
        is_finished = False
        while not is_finished:
            # Issue the navigation command about twice a second such that it is easy to terminate the
            # navigation command (with estop or killing the program).
            nav_to_cmd_id = self._graph_nav_client.navigate_to(destination_waypoint, 1.0,
                                                               leases=[sublease.lease_proto])
            time.sleep(.5)  # Sleep for half a second to allow for command execution.
            # Poll the robot for feedback to determine if the navigation command is complete. Then sit
            # the robot down once it is finished.
            is_finished = self._check_success(nav_to_cmd_id)

        self._lease = self._lease_wallet.advance()
        self._lease_keepalive = LeaseKeepAlive(self._lease_client)

    def _start_navigation_route(self):
        cWayPts = self._current_graph.waypoints
        allWayPtIDs = []

        def addWPtFunc(idx):
            cName = "waypoint" + str(idx)
            cPt = self._getWayPtWithName(cName)
            allWayPtIDs.append(cPt.id)
            self.runRobotLog("Adding waypoint %s to Navigation Path" %cName)            

        # Add pairs of waypoints for traversal
        for i in range(0, len(cWayPts)):
            addWPtFunc(i)
            
        #Localize to 1st Waypoint
        self._set_initial_localization_waypoint(allWayPtIDs[0])
        self._get_localization_state()
        # Now Prepare for Navigation
        self._prepare_navigate_route(allWayPtIDs)
    
    def _can_edge_run(self, waypoint_ids):
        edge_ids_list = []
        # Attempt to find edges in the current graph that match the ordered waypoint pairs.
        # These are necessary to create a valid route.
        for i in range(len(waypoint_ids) - 1):
            start_wp = waypoint_ids[i]
            end_wp = waypoint_ids[i + 1]
            edge_id = self._match_edge(self._current_edges, start_wp, end_wp)
            if edge_id is not None:
                edge_ids_list.append(edge_id)
            else:
                print("Failed to find an edge between waypoints: ", start_wp, " and ", end_wp)
                self.runRobotLog(
                    "List the graph's waypoints and edges to ensure pairs of waypoints has an edge."
                )
                return []
                
        return edge_ids_list

    def _prepare_navigate_route(self, waypoint_ids):
        """Navigate through a specific route of waypoints."""
        self._lease = self._lease_wallet.get_lease()
        # Stop the lease keepalive and create a new sublease for graph nav.
        self._lease = self._lease_wallet.advance()
        self._sublease = self._lease.create_sublease()
        self._lease_keepalive.shutdown()

        # Navigate a specific route.
        if self._nav_run_mode == SPOT_NAV_RUN_EDGE_MODE:        
            edge_ids_list = self._can_edge_run(waypoint_ids)
            if len(edge_ids_list) > 0:
                self._route = self._graph_nav_client.build_route(waypoint_ids, edge_ids_list)

        self._route_is_finished = False
        self._routeTime = self._getTimeMS()
        self._routeNavIdx =  0
        self._routeNavWayPts = waypoint_ids
        self.runRobotLog("Robot navigation prepped and ready for %d waypoints" %len(waypoint_ids))
        # Setup appropriate route function tick callbacks
        self._routeFnMap = {}
        self._routeFnMap[SPOT_NAV_RUN_EDGE_MODE] = self._tick_navigate_routeEdges
        self._routeFnMap[SPOT_NAV_RUN_PTS_MODE] = self._tick_navigate_routePts
    
    def _tick_navigate_route(self):
        self._routeFnMap[self._nav_run_mode]()

    def _can_tick_route(self):
        if self._route_is_finished:
            self._end_navigate_route()
            return False

        cTime = self._getTimeMS()
        diffTime = cTime - self._routeTime
        if(diffTime < 500):
            # Only allow an actual navigation command every 0.5 seconds
            return False
        self._routeTime = cTime
        return True

    def _tick_navigate_routePts(self):
        ''' This navigates via a list of waypoint IDs '''
        if self._can_tick_route() == False:
            return False

        try:
            # Issue the route command about twice a second such that it is easy to terminate the
            # navigation command (with estop or killing the program).
            nav_to_cmd_id = self._graph_nav_client.navigate_to(
                self._routeNavWayPts[self._routeNavIdx], cmd_duration=1.0, leases=[self._sublease.lease_proto])
            # Poll the robot for feedback to determine if the route is complete.
            if self._check_success(nav_to_cmd_id):
                self.runRobotLog("Robot hit waypoint%d" %(self._routeNavIdx))
                self._routeNavIdx += 1
                if self._routeNavIdx >= len(self._routeNavWayPts):
                    self.runRobotLog("Robot navigation finished last waypoint.")
                    self._route_is_finished = True
            
            return True
        except Exception as e:
            self.runRobotLog("GraphNav Error:")
            self.runRobotLog(e)
            return False        

        return False    

    def _tick_navigate_routeEdges(self):
        ''' This navigates with a route defined using the current Edge snapshot'''
        if self._can_tick_route() == False:
            return

        try:
            # Issue the route command about twice a second such that it is easy to terminate the
            # navigation command (with estop or killing the program).
            nav_route_command_id = self._graph_nav_client.navigate_route(
                self._route, cmd_duration=1.0, leases=[self._sublease.lease_proto])
            # Poll the robot for feedback to determine if the route is complete.
            self._route_is_finished = self._check_success(nav_route_command_id)
            return True
        except Exception as e:
            self.runRobotLog("GraphNav Error:")
            self.runRobotLog(e)
            return False

    def _end_navigate_route(self):
        self._lease = self._lease_wallet.advance()
        self._lease_keepalive = LeaseKeepAlive(self._lease_client)
        self.runRobotLog("Finished Auto Navigation Route.")

    def _clear_graph(self, *args):
        """Clear the state of the map on the robot, removing all waypoints and edges."""
        self._lease = self._lease_wallet.advance()
        return self._graph_nav_client.clear_graph(lease=self._lease.lease_proto)                      