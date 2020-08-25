import grpc
import logging
import os
import glob
import sys
import time

from bosdyn.api.graph_nav import recording_pb2
from bosdyn.client import create_standard_sdk, ResponseError, RpcError
import bosdyn.client.channel
from bosdyn.client.graph_nav import GraphNavClient
from bosdyn.client.lease import LeaseClient
from bosdyn.client.recording import GraphNavRecordingServiceClient
import bosdyn.client.util
import google.protobuf.timestamp_pb2

class SpotNavRecord(object):
    """ Allows you to Record a fixed Waypoint Path using Spot's Autonomy API """
    def __init__(self, robot, graphFolder):
        self._robotLogFN = None
        self._robot = robot
        self._robot.time_sync.wait_for_sync()
        
        # Where to download the navGraphs
        self._graphFolder = graphFolder
        os.makedirs(self._graphFolder, exist_ok=True)

        # Setup the recording service client.
        self._recording_client = self._robot.ensure_client(
            GraphNavRecordingServiceClient.default_service_name)
        
        # Setup the graph nav service client.
        self._graph_nav_client = robot.ensure_client(GraphNavClient.default_service_name)

    def setRobotLogFN(self, fnIn):
        self._robotLogFN = fnIn

    def runRobotLog(self, msgIn):
        print(msgIn)
        if self._robotLogFN:
            self._robotLogFN(msgIn)

    @staticmethod
    def _getAllGraphFiles(graphFolderIn, extStr="spotGraphFolder"):
        retFiles = []
        for file in os.listdir(graphFolderIn):
            if file.endswith(extStr):
                retFiles.append(os.path.join(graphFolderIn, file))
        return retFiles

    @staticmethod
    def _getUniqueGraphFolder(graphFolderIn):
        cnt = 0
        folderExt = "spotGraphFolder"
        allFolders = SpotNavRecord._getAllGraphFiles(graphFolderIn, folderExt)
        while True:
            rFolder = graphFolderIn + "/navGraph_" + str(cnt) + "." + folderExt
            if (rFolder in allFolders) == False:
                return rFolder
            cnt += 1
        return ""

    @staticmethod
    def _getGraphDataFilename(idx, folderIn, extStr):
        return folderIn + "/navGraph_" + str(idx) + "." + extStr

    def _write_bytes(self, filename, data):
        with open(filename, 'wb+') as f:
            f.write(data)
            f.close()

    def _clear_graph(self, lease_client):
        """Clear the state of the map on the robot, removing all waypoints and edges."""
        lease_wallet = lease_client.lease_wallet
        cLease = lease_wallet.advance()
        retval = self._graph_nav_client.clear_graph(lease=cLease)                   
        print("Clearing the state of the map on the robot, removing all waypoints and edges.")
        return retval

    def should_we_start_recording(self):
        # Before starting to record, check the state of the GraphNav system.
        graph = self._graph_nav_client.download_graph()
        if graph is not None:
            # Check that the graph has waypoints. If it does, then we need to be localized to the graph
            # before starting to record
            if len(graph.waypoints) > 0:
                localization_state = self._graph_nav_client.get_localization_state()
                if not localization_state.localization.waypoint_id:
                    # Not localized to anything in the map. The best option is to clear the graph or
                    # attempt to localize to the current map.
                    # Returning false since the GraphNav system is not in the state it should be to
                    # begin recording.
                    return False
        # If there is no graph or there exists a graph that we are localized to, then it is fine to
        # start recording, so we return True.
        return True

    def _start_recording(self):
        """Start recording a map."""
        should_start_recording = self.should_we_start_recording()
        if not should_start_recording:
            print("The system is not in the proper state to start recording.", \
                   "Try using the graph_nav_command_line to either clear the map or", \
                   "attempt to localize to the map.")
            return
        try:
            status = self._recording_client.start_recording()
            self.runRobotLog("Successfully started recording a map.")
        except Exception as err:
            self.runRobotLog("Start recording failed: "+str(err))

    def _stop_recording(self):
        """Stop or pause recording a map."""
        try:
            status = self._recording_client.stop_recording()
            self.runRobotLog("Successfully stopped recording a map.")
        except Exception as err:
            self.runRobotLog("Stop recording failed: "+str(err))

    def _get_recording_status(self):
        """Get the recording service's status."""
        status = self._recording_client.get_record_status()
        if status.is_recording:
            self.runRobotLog("The recording service is on.")
        else:
            self.runRobotLog("The recording service is off.")

    def _create_default_waypoint(self):
        """Create a default waypoint at the robot's current location."""
        resp = self._recording_client.create_waypoint(waypoint_name="default")
        if resp.status == recording_pb2.CreateWaypointResponse.STATUS_OK:
            self.runRobotLog("Successfully created a waypoint.")
        else:
            self.runRobotLog("Could not create a waypoint.")

    def _download_full_graph(self):
        """Download the graph and snapshots from the robot."""
        newFolder = SpotNavRecord._getUniqueGraphFolder(self._graphFolder)
        os.mkdir(newFolder)
        self.runRobotLog("Created new folder for Recording: %s" %newFolder)

        graph = self._graph_nav_client.download_graph()
        if graph is None:
            self.runRobotLog("Failed to download the graph.")
            return
        self._write_full_graph(graph, newFolder)
        self.runRobotLog("Graph downloaded with {} waypoints and {} edges".format(
            len(graph.waypoints), len(graph.edges)))
        # Download the waypoint and edge snapshots.
        self._download_and_write_waypoint_snapshots(graph.waypoints, newFolder)
        self._download_and_write_edge_snapshots(graph.edges, newFolder)

    def _write_full_graph(self, graph, folderIn):
        """Download the graph from robot to the specified, local filepath location."""
        graph_bytes = graph.SerializeToString()
        cGraphFilename = SpotNavRecord._getGraphDataFilename(0, folderIn, "spotGraph")
        self._write_bytes(cGraphFilename, graph_bytes)
        self.runRobotLog("Wrote graph to file: %s" %cGraphFilename)

    def _download_and_write_waypoint_snapshots(self, waypoints, folderIn):
        """Download the waypoint snapshots from robot to the specified, local filepath location."""
        num_waypoint_snapshots_downloaded = 0
        for waypoint in waypoints:
            try:
                waypoint_snapshot = self._graph_nav_client.download_waypoint_snapshot(
                    waypoint.snapshot_id)
            except Exception:
                # Failure in downloading waypoint snapshot. Continue to next snapshot.
                self.runRobotLog("Failed to download waypoint snapshot: " + waypoint.snapshot_id)
                continue

            cWayptsFilename = SpotNavRecord._getGraphDataFilename(num_waypoint_snapshots_downloaded, folderIn, "spotWaypts")
            self._write_bytes(cWayptsFilename, waypoint_snapshot.SerializeToString())
            num_waypoint_snapshots_downloaded += 1
            self.runRobotLog("Downloaded {} of the total {} waypoint snapshots.".format(
                num_waypoint_snapshots_downloaded, len(waypoints)))

    def _download_and_write_edge_snapshots(self, edges, folderIn):
        """Download the edge snapshots from robot to the specified, local filepath location."""
        num_edge_snapshots_downloaded = 0
        for edge in edges:
            try:
                edge_snapshot = self._graph_nav_client.download_edge_snapshot(edge.snapshot_id)
            except Exception:
                # Failure in downloading edge snapshot. Continue to next snapshot.
                self.runRobotLog("Failed to download edge snapshot: " + edge.snapshot_id)
                continue

            cEdgesFilename = SpotNavRecord._getGraphDataFilename(num_edge_snapshots_downloaded, folderIn, "spotEdges")
            self._write_bytes(cEdgesFilename, edge_snapshot.SerializeToString())
            num_edge_snapshots_downloaded += 1
            self.runRobotLog("Downloaded {} of the total {} edge snapshots.".format(
                num_edge_snapshots_downloaded, len(edges)))
