"""WASD driving of robot."""

from __future__ import print_function
from collections import OrderedDict
import curses
import logging
import signal
import sys
import threading
import time
import spotMsgs
import json
import msgpack
from spotNavRecord import SpotNavRecord
from spotNavRun import SpotNavRun
import spotNavRun
from PIL import Image, ImageEnhance
import datetime
import os
import cv2

from bosdyn.api import geometry_pb2
import bosdyn.api.power_service_pb2 as PowerServiceProto
# import bosdyn.api.robot_command_pb2 as robot_command_pb2
import bosdyn.api.robot_state_pb2 as robot_state_proto
import bosdyn.api.spot.robot_command_pb2 as spot_command_pb2
from bosdyn.api import power_pb2
from bosdyn.api import trajectory_pb2
from bosdyn.client import create_standard_sdk, ResponseError, RpcError
from bosdyn.client.async_tasks import AsyncGRPCTask, AsyncPeriodicQuery, AsyncTasks
from bosdyn.client.estop import EstopClient, EstopEndpoint, EstopKeepAlive
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
from bosdyn.client.power import PowerClient
from bosdyn.client.robot_command import RobotCommandClient, RobotCommandBuilder
from bosdyn.client.image import ImageClient
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.time_sync import TimeSyncError
from bosdyn.client.robot_command import RobotCommandClient, blocking_stand
from bosdyn.client.frame_helpers import get_odom_tform_body
import bosdyn.client.util
import bosdyn.geometry
import bosdyn.client.image
from bosdyn.util import duration_str, format_metric, secs_to_hms
import spotCam
import spotCamStream

VELOCITY_BASE_SPEED = 0.5  # m/s
VELOCITY_BASE_ANGULAR = 0.8  # rad/sec
VELOCITY_CMD_DURATION = 0.6  # seconds
VELOCITY_CMD_INIT_DURATION = VELOCITY_CMD_DURATION  #* 2  # seconds
COMMAND_INPUT_RATE = 50
#ACTIVE_SERVICE = None

class SpotControlState(object):
    def __init__(self):
        self.px = None
        self.py = None
        self.pz = None

        self.qx = None
        self.qy = None
        self.qz = None
        self.qw = None

        self.moveVX = None
        self.moveVY = None
        self.moveVROT = None

        self.cameraSpotCamImage = None
        self.cameraSpotCamActive = True

        self.stateLock = threading.Lock()
        self.packArray = []
    
    def setXform(self, x, y, z, qx, qy, qz, qw):
        with self.stateLock:
            self.px = x
            self.py = y
            self.pz = z

            self.qx = qx
            self.qy = qy
            self.qz = qz
            self.qw = qw

    def setMoveVel(self, vx, vy, vrot):
        with self.stateLock:
            self.moveVX = vx
            self.moveVY = vy
            self.moveVROT = vrot

    def setCameraSpotCam(self, img):
        with self.stateLock:
            self.cameraSpotCamImage = img

    def isValid(self):
        state1_OK = (self.px != None) and (self.py != None) and (self.pz != None) and (self.qx != None) and (self.qy != None) and (self.qz != None) and (self.qw != None)
        state2_OK = (self.moveVX != None) and (self.moveVY != None) and (self.moveVROT != None)
        return state1_OK and state2_OK

    @staticmethod
    def getUniqueFolder(baseFolder):
        def _getAllFiles(folderIn, extStr):
            retFiles = []
            for file in os.listdir(folderIn):
                if file.endswith(extStr):
                    retFiles.append(os.path.join(folderIn, file))
            return retFiles

        cnt = 0
        folderExt = "stateRun"
        allFolders = _getAllFiles(baseFolder, folderExt)
        while True:
            rFolder = baseFolder + "/record_" + str(cnt) + "." + folderExt
            if (rFolder in allFolders) == False:
                return rFolder
            cnt += 1
        return ""

    def getCurPackBin(self):
        with self.stateLock:
            if self.isValid():
                cPackArray = [self.px, self.py, self.pz, self.qx, self.qy, self.qz, self.qw, self.moveVX, self.moveVY, self.moveVROT]
                if self.cameraSpotCamActive and (self.cameraSpotCamImage != None):
                    cPackArray.extend([self.cameraSpotCamImage])
                cPackArray.extend([str(datetime.datetime.now().time())])
                packData = msgpack.packb(cPackArray, use_bin_type=True)
                return packData
        return None
        
    def packToBin(self):
        packData = msgpack.packb(self.packArray, use_bin_type=True)
        print("Control State has %d samples." %(len(self.packArray) / 10))
        return packData

SPOT_RUN_NORMAL_MODE = 0
SPOT_RUN_NAV_RECORD_MODE = 1
SPOT_RUN_NAV_PLAY_MODE = 2
SPOT_RUN_NAV_PROCESS_PLAY_MODE = 3

class SpotRobot(object):
    def __init__(self, appTokenPath, hostname, username, password, cameraQuality=75, registerServices=False):
        self._spotMode = SPOT_RUN_NORMAL_MODE
        #bosdyn.client.util.setup_logging(True)
        self.sdk = bosdyn.client.create_standard_sdk('HelloSpotClient')

        if registerServices:
            from bosdyn.client import spot_cam
            spot_cam.register_all_service_clients(self.sdk)
            print("Registering Spot Services.")

        self.sdk.load_app_token(appTokenPath)
        self.robot = self.sdk.create_robot(hostname)

        # Clients need to authenticate to a robot before being able to use it.
        self.robot.authenticate(username, password)

        self._robot_command_client = self.robot.ensure_client(RobotCommandClient.default_service_name)
        self._robot_state_client =  self.robot.ensure_client(RobotStateClient.default_service_name)

        # Spot requires a software estop to be activated.
        self.estop_client = self.robot.ensure_client(bosdyn.client.estop.EstopClient.default_service_name)
        self.estop_endpoint = bosdyn.client.estop.EstopEndpoint(client=self.estop_client, name='HelloSpot',
                                                        estop_timeout=9.0)
        self.estop_endpoint.force_simple_setup()
        # Keep Alive estop
        self.estopAlive = bosdyn.client.estop.EstopKeepAlive(self.estop_endpoint)

        # Only one client at a time can operate a robot. Clients acquire a lease to
        # indicate that they want to control a robot. Acquiring may fail if another
        # client is currently controlling the robot. When the client is done
        # controlling the robot, it should return the lease so other clients can
        # control it. Note that the lease is returned as the "finally" condition in this
        # try-catch-finally block.
        self.lease_client = self.robot.ensure_client(bosdyn.client.lease.LeaseClient.default_service_name)
        # Keep Alive Lease then acquire
        self.leaseAlive = bosdyn.client.lease.LeaseKeepAlive(self.lease_client)
        self.lease = self.lease_client.take()

        # Establish time sync with the robot. This kicks off a background thread to establish time sync.
        # Time sync is required to issue commands to the robot. After starting time sync thread, block
        # until sync is established.
        self.robot.time_sync.wait_for_sync()

        # Image Client setup
        self.image_client = self.robot.ensure_client(ImageClient.default_service_name)

        self.stateTime = self.current_milli_time()
        self.robotIsOn = False
        self.mobilityParams = None
        self.cameraQuality = cameraQuality
        self.alwaysPublishOdom = False

        self.camSourceNames = []
        self.captureDepth = False
        self.camDepthNameMap = {}
        self.camDepthNameMap["back_fisheye_image"] = "back_depth"
        self.camDepthNameMap["frontleft_fisheye_image"] = "frontleft_depth"
        self.camDepthNameMap["frontright_fisheye_image"] = "frontright_depth"
        self.camDepthNameMap["left_fisheye_image"] = "left_depth"
        self.camDepthNameMap["right_fisheye_image"] = "right_depth"

        self.msgLock = threading.Lock()
        self.msgWebsockets = []

        # Setup command function table/map
        self.defineCommandMap()

        # Store control state
        self.controlState = SpotControlState()
        self.controlStateRecordFlag = False
        self.controlStateRecordFolder = None

        # Spot NavGraph
        self.navRecord = None
        self.navRun = None
        self.navMode = spotNavRun.SPOT_NAV_RUN_PTS_MODE

        # Spot Cam
        self.spotCamStream = None
        self.spotCamFolder = ""

    def spotSetLighting(self, flagIn):
        spotCam.SpotCam.setLighting(self.robot, flagIn)

    def spotCamSnapShot(self):
        def snapFinish():
            del self.snapCam
            print("Removing SpotSnapCam")

        self.snapCam = spotCam.SpotCam(self.spotCamFolder, self.robot, snapFinish)

    def spotCamStart(self, storeFolder, sHostName):
        self.spotCamFolder = storeFolder
        self.spotCamHostname = sHostName
        self.spotCamRestart()

    def spotCamRestart(self):
        self.spotCamStream = spotCamStream.WebRTCStreamer(self.spotCamFolder, self.spotCamHostname)
        self.spotCamStream.run(self.robot)
    
    def spotCamStop(self):
        if self.spotCamStream != None:
            self.spotCamStream.stopStream()

    def spotCamRecord(self, flagIn):
        if self.spotCamStream == None:
            return

        self.spotCamStream.setRecord(flagIn)
    
    def spotCamIsRecording(self):
        if self.spotCamStream == None:
            return False
        
        return self.spotCamStream.isRecording()
    
    def setNavRecordFolder(self, graphFolderIn):
        self.navFolder = graphFolderIn

    def setupNavRecord(self, graphFolderIn, sServiceIn, websocket):
        self.navRecord = SpotNavRecord(self.robot, graphFolderIn)
        
        def cLogFn(msgIn):
            self.sendBackRobotLogMsg(msgIn, sServiceIn, websocket)
        
        self.navRecord.setRobotLogFN(cLogFn)

    def setNavRunMode(self, modeStr):
        if modeStr == "pts":
             self.navMode = spotNavRun.SPOT_NAV_RUN_PTS_MODE
        elif modeStr == "edge":
            self.navMode = spotNavRun.SPOT_NAV_RUN_EDGE_MODE
        else:
            print("Invalid nav mode: %s . Should be either 'pts' or 'edge' ." %modeStr)        

    def setupNavRun(self, graphFolderIn, graphFilenameIn, sServiceIn, websocket):
        self._spotMode = SPOT_RUN_NAV_PROCESS_PLAY_MODE
        self.navRun = SpotNavRun(
            self.robot, 
            self._robot_state_client, 
            self.lease_client, 
            self.leaseAlive,
            graphFolderIn, 
            graphFilenameIn,
            self.navMode)

        def cLogFn(msgIn):
            self.sendBackRobotLogMsg(msgIn, sServiceIn, websocket)
        
        self.navRun.setRobotLogFN(cLogFn)
        print("Switching to GraphNav Play mode with file: %s" %graphFilenameIn)

        def navRunSetupFn():
            self.navRun._clear_graph()
            self.navRun._upload_graph_and_snapshots()
            self.navRun._list_graph_waypoint_and_edge_ids()
            self.navRun._start_navigation_route()
            self._spotMode = SPOT_RUN_NAV_PLAY_MODE

        navSetupThread = threading.Thread(target=navRunSetupFn)
        navSetupThread.start()
    
    def clearNavGraph(self):
        cNavRun = SpotNavRun(
            self.robot, 
            self._robot_state_client, 
            self.lease_client, 
            self.leaseAlive,
            None, 
            None)
        cNavRun._clear_graph()        
    
    def endNavRun(self):
        self.navRun._end_navigate_route()
        self.navRun = None
        self._spotMode = SPOT_RUN_NORMAL_MODE

    def current_milli_time(self):
        return int(round(time.time() * 1000))

    def __del__(self):
        self.shutdown()

    def shutdown(self, forceSit=False):
        self.spotCamStop()
        if forceSit:
            robotState = self._robot_state_client.get_robot_state()
            if robotState.power_state.motor_power_state == robot_state_proto.PowerState.STATE_ON:                
                print("Trying to Shutdown Server so getting the robot to sit first...")
                self._start_robot_command("sit", RobotCommandBuilder.sit_command())
                print("Now waiting for 15 seconds...")
                time.sleep(15)

        self.estopAlive.stop()
        self.estopAlive.shutdown()
        self.lease_client.return_lease(self.lease)

    def controlStateStartRecord(self):
        if self.alwaysPublishOdom == False:
            print("Cannot start Control State Record because alwaysPublishOdom is False!")
            return

        self.controlStateRecordFlag = True
        self.controlStateRecordWriteFolder = SpotControlState.getUniqueFolder(self.controlStateRecordFolder)
        os.mkdir(self.controlStateRecordWriteFolder)
        self.controlStateCnt = 0
        print("Starting Control State Record to folder: %s" %(self.controlStateRecordWriteFolder))

    def controlStateEndRecord(self):
        print("Ending Control State Record, wrote out %d state objects." %self.controlStateCnt)
        self.controlStateRecordFlag = False
    
    def controlStateTick(self):
        if self.controlStateRecordFlag == False:
            return
        wBin = self.controlState.getCurPackBin()
        if(wBin != None):
            f = open(self.controlStateRecordWriteFolder + "/data_" + str(self.controlStateCnt) + ".pack", 'wb')
            f.write(wBin)
            self.controlStateCnt += 1

    def parseSpotNavCmd(self, cmdPacket, sServiceIn, websocket):
        jsonObj = cmdPacket.getDataMap()
        print("Spot GraphNav Packet received: %s" %(cmdPacket.msg))
        if self.navFolder == None:
            print("Cannot process NavRecord! No 'navGraphFolder' defined in spotServerConfig.json!")
            return

        def returnNavFilesPkt():
            allFiles = SpotNavRecord._getAllGraphFiles(self.navFolder)
            retData = {}
            retData["files"] = allFiles
            retPkt = spotMsgs.SpotGraphNavCommandPacket()
            retPkt.msg = json.dumps(retData)
            sServiceIn.addRespPacket(retPkt, websocket)            

        if "cmd" in jsonObj:
            cCmd = jsonObj["cmd"]

            if cCmd == "list":
                # Return all available recorded graphs
                print("List all GraphNav files...")
                returnNavFilesPkt()
            elif cCmd == "recordStart":
                # Start a new GraphNav recording
                self.setupNavRecord(self.navFolder, sServiceIn, websocket)
                print("Starting GraphNav Record...")
                #self.navRecord._clear_graph(self.lease_client)
                self.clearNavGraph()
                self.navRecord._start_recording()
                self._spotMode = SPOT_RUN_NAV_RECORD_MODE                    
            elif cCmd == "recordStop":
                # Stop the current GraphNav recording and download results to disk
                print("Stopping GraphNav Record...")
                self.navRecord._stop_recording()
                self.navRecord._download_full_graph()
                self.navRecord = None
                self._spotMode = SPOT_RUN_NORMAL_MODE
                returnNavFilesPkt()
            elif (cCmd == "playNav") and ("navFile" in jsonObj):
                # Play an existing GraphNav file
                allFiles = SpotNavRecord._getAllGraphFiles(self.navFolder)
                playFile = jsonObj["navFile"]
                if (playFile in allFiles) == False:
                    print("Error! GraphNav File cannot be played: %s" %playFile)
                    return

                try:
                    self.setupNavRun(self.navFolder, playFile, sServiceIn, websocket)
                except Exception as e:
                     self.sendBackRobotLogMsg(str(e), sServiceIn, websocket)

            elif (cCmd == "stopNav") and (self._spotMode == SPOT_RUN_NAV_PLAY_MODE):
                self.endNavRun()
    
    def defineCommandMap(self):
        self.cmdMap = {}

        def cmdBytesFn(cmdPacket, sServiceIn, websocket):
            print("Bytes Command Packet with size: %d" %len(cmdPacket.bytes))
            return None
        
        def cmdMoveVelFn(cmdPacket, sServiceIn, websocket):
            self.runMoveVel(cmdPacket)
            return None
        
        def cmdPowerRobotFn(cmdPacket, sServiceIn, websocket):
            self.powerRobot(cmdPacket.state)
            return None
        
        def cmdRobotCommandFn(cmdPacket, sServiceIn, websocket):
            self.runRobotCommand(cmdPacket)
            return None
        
        def cmdImageStreamFn(cmdPacket, sServiceIn, websocket):
            if len(cmdPacket.imageNames) == 1:
                if cmdPacket.imageNames[0] == "spot_cam":
                    return self.spotCamSendStream(websocket)

            return self.captureImages(sServiceIn, websocket, cmdPacket.imageNames)

        def cmdMotionParamsFn(cmdPacket, sServiceIn, websocket):
            return self.setMotionParams(cmdPacket)
        
        def cmdMsgStrFn(cmdPacket, sServiceIn, websocket):
            if cmdPacket.msg == "status":
                print("Retrieving robot state...")
                return self.retrieveRobotState(sServiceIn, websocket)
            elif cmdPacket.msg == "take":
                print("Trying to take robot")
                self.attemptGrabLease()
            elif cmdPacket.msg == "odom":
                self.retrieveRobotOdomState(sServiceIn, websocket, None)
            elif cmdPacket.msg == "odomPublishON":
                self.alwaysPublishOdom = True
            elif cmdPacket.msg == "odomPublishOFF":
                self.alwaysPublishOdom = False
            elif cmdPacket.msg == "controlStateStartRecord":
                self.controlStateStartRecord()
            elif cmdPacket.msg == "controlStateEndRecord":
                self.controlStateEndRecord()
            elif cmdPacket.msg == "clearWaypoints":
                self.relayRespPacket(cmdPacket, sServiceIn, websocket)
            elif cmdPacket.msg == "spotCamRecordStart":
                self.spotCamRecord(True)
            elif cmdPacket.msg == "spotCamRecordStop":
                self.spotCamRecord(False)
            elif cmdPacket.msg == "spotCamSnap":
                self.spotCamSnapShot()
            elif cmdPacket.msg == "spotCamRestart":
                self.spotCamRestart()
            elif cmdPacket.msg == "spotLightON":
                self.spotSetLighting(True)
            elif cmdPacket.msg == "spotLightOFF":
                self.spotSetLighting(False)
            return None

        def cmdSettingsFn(cmdPacket, sServiceIn, websocket):
            jsonObj = cmdPacket.getDataMap()
            print("Updating Robot Settings: %s" %(cmdPacket.msg))
            if "cameraQuality" in jsonObj:
                self.cameraQuality = int(jsonObj["cameraQuality"])

        def cmdRelayRespPacketFn(cmdPacket, sServiceIn, websocket):
            self.relayRespPacket(cmdPacket, sServiceIn, websocket)
            return None

        def cmdSpotGraphNavPacketFn(cmdPacket, sServiceIn, websocket):
            self.parseSpotNavCmd(cmdPacket, sServiceIn, websocket)            

        self.cmdMap[spotMsgs.COMMAND_MSG_TYPE] = cmdMsgStrFn
        self.cmdMap[spotMsgs.COMMAND_BYTES_TYPE] = cmdBytesFn
        self.cmdMap[spotMsgs.COMMAND_MOVE_VEL_TYPE] = cmdMoveVelFn
        self.cmdMap[spotMsgs.COMMAND_POWER_ROBOT_TYPE] = cmdPowerRobotFn
        self.cmdMap[spotMsgs.COMMAND_ROBOT_CMD_TYPE] = cmdRobotCommandFn
        self.cmdMap[spotMsgs.COMMAND_IMG_TYPE] = None
        self.cmdMap[spotMsgs.COMMAND_IMG_STREAM_TOGGLE_TYPE] = cmdImageStreamFn
        self.cmdMap[spotMsgs.COMMAND_IMG_LIST_TYPE] = None
        self.cmdMap[spotMsgs.COMMAND_STATUS_TYPE] = None
        self.cmdMap[spotMsgs.COMMAND_MOTION_PARAMS_TYPE] = cmdMotionParamsFn
        self.cmdMap[spotMsgs.COMMAND_SETTINGS_TYPE] = cmdSettingsFn
        self.cmdMap[spotMsgs.COMMAND_WAYPOINTS_TYPE] = cmdRelayRespPacketFn
        self.cmdMap[spotMsgs.COMMAND_MOVE_WAYPOINT_TYPE] = cmdRelayRespPacketFn
        self.cmdMap[spotMsgs.COMMAND_SPOT_NAVGRAPH_TYPE] = cmdSpotGraphNavPacketFn
        self.cmdMap[spotMsgs.COMMAND_UPDATE_POSE_TYPE] = cmdRelayRespPacketFn
        self.cmdMap[spotMsgs.COMMAND_INITIAL_POSITION_TYPE] = cmdRelayRespPacketFn
    
    def execCommandFn(self, cmdPacket, sServiceIn, websocket):
        '''
        try:
            execFn = self.cmdMap[cmdPacket.getPacketTypeID()]
            return execFn(cmdPacket, sServiceIn)
        except:
            print("ERROR! Cannot execute command function for packet: %s" %(str(cmdPacket)))
        
        return None
        '''
        execFn = self.cmdMap[cmdPacket.getPacketTypeID()]
        return execFn(cmdPacket, sServiceIn, websocket)

    def tickNavRun(self):
        retval = self.navRun._tick_navigate_route()
        if retval == False:
            # Navigation Failed
            print("Ending Navigation due to Failure")
            self.endNavRun()
        elif self.navRun._route_is_finished:
            # Navigation Done
            self.endNavRun()

    def tick(self, cmdPacket, sServiceIn, websocket):
        # Rate Limit
        isImageRequest = isinstance(cmdPacket, spotMsgs.ImageStreamTogglePacket)
        if(isImageRequest == False):
            curStateTime = self.current_milli_time()
            diffStateTime = curStateTime - self.stateTime
            if(diffStateTime < COMMAND_INPUT_RATE):
                return None
            self.stateTime = curStateTime

        if self.alwaysPublishOdom:
            self.retrieveRobotOdomState(sServiceIn, websocket)

        self.controlStateTick()

        if self._spotMode == SPOT_RUN_NAV_PLAY_MODE:
            self.tickNavRun()

        if (self._spotMode == SPOT_RUN_NAV_PROCESS_PLAY_MODE) and (isImageRequest == False):
            return None

        return self.execCommandFn(cmdPacket, sServiceIn, websocket)

    def spotMobilityParams(
        self,
        body_height=0.0, 
        footprint_R_body=bosdyn.geometry.EulerZXY(),
        locomotion_hint=spot_command_pb2.HINT_AUTO, 
        stair_hint=False,
        obstacle_avoidance=True,
        external_force_params=None):
            """Helper to create Mobility params for spot mobility commands. This function is designed
            to help get started issuing commands, but lots of options are not exposed via this
            interface. See spot.robot_command_pb2 for more details. If unset, good defaults will be
            chosen by the robot.

            Returns:
                spot.MobilityParams, params for spot mobility commands.
            """
            # Simplified body control params
            position = geometry_pb2.Vec3(z=body_height)
            rotation = footprint_R_body.to_quaternion()
            pose = geometry_pb2.SE3Pose(position=position, rotation=rotation)
            point = trajectory_pb2.SE3TrajectoryPoint(pose=pose)
            traj = trajectory_pb2.SE3Trajectory(points=[point])
            body_control = spot_command_pb2.BodyControlParams(base_offset_rt_footprint=traj)
            obstacles = None
            if(obstacle_avoidance == False):
                obstacles = spot_command_pb2.ObstacleParams(disable_vision_body_obstacle_avoidance=True,
                                                    disable_vision_foot_obstacle_avoidance=True,
                                                    disable_vision_foot_constraint_avoidance=True,
                                                    obstacle_avoidance_padding=.001)
                return spot_command_pb2.MobilityParams(body_control=body_control,
                                                       locomotion_hint=locomotion_hint,
                                                       stair_hint=stair_hint,
                                                       obstacle_params=obstacles)        



            return spot_command_pb2.MobilityParams(body_control=body_control,
                                                   locomotion_hint=locomotion_hint,
                                                   stair_hint=stair_hint)        

    def setMotionParams(self, cmdPacket):
        mParams = cmdPacket.getDataMap()

        locomotion_hint = spot_command_pb2.HINT_AUTO
        if "locomotion_hint" in mParams:
            locHintStr = mParams["locomotion_hint"]
            if locHintStr == "HINT_AUTO":
                locomotion_hint = spot_command_pb2.HINT_AUTO
            elif locHintStr == "HINT_CRAWL":
                locomotion_hint = spot_command_pb2.HINT_CRAWL
        
        body_height = 0.0
        if "body_height" in mParams:
            body_height = float(mParams["body_height"])

        yaw = 0.0
        roll = 0.0
        pitch = 0.0
        if ("yaw" in mParams) and ("roll" in mParams) and ("pitch" in mParams):
            yaw = float(mParams["yaw"])
            roll = float(mParams["roll"])
            pitch = float(mParams["pitch"])

        start_hint = False
        if "stair_hint" in mParams:
            stair_hint = bool(mParams["stair_hint"])

        obstacle_avoidance = True
        if "obstacle_avoidance" in mParams:
            obstacle_avoidance = bool(mParams["obstacle_avoidance"])

        self.mobilityParams = self.spotMobilityParams(
            body_height=body_height, 
            footprint_R_body=bosdyn.geometry.EulerZXY(yaw, roll, pitch),
            locomotion_hint=locomotion_hint,
            stair_hint=stair_hint,
            obstacle_avoidance=obstacle_avoidance)

        # The input params can also take additional robot commands if required
        if "robotCmd" in mParams:
            if mParams["robotCmd"] == "stand":
                self._start_robot_command(cmdPacket.msg, RobotCommandBuilder.stand_command(params=self.mobilityParams))

        return None

    def getDepthCamName(self, regularCamName):
        return self.camDepthNameMap[regularCamName]
    
    def isDepthCamName(self, camNameIn):
        return camNameIn.endswith("depth")

    def attemptGrabLease(self):
        self.lease = self.lease_client.take()

    def retrieveRobotOdomState(self, sServiceIn, websocket, robotState=None):
        if robotState == None:
            robotState = self._robot_state_client.get_robot_state()
        #Encode Odometry
        robotXform = get_odom_tform_body(
            robotState.kinematic_state.transforms_snapshot).to_proto()
        robotVform = robotState.kinematic_state.velocity_of_body_in_odom
        retXformPkt = spotMsgs.XformCommandPacket.createXformPacket(
            0, # Subtype, 0 for default root 
            robotXform.position.x, # (m)
            robotXform.position.y,
            robotXform.position.z,
            robotXform.rotation.x,
            robotXform.rotation.y,
            robotXform.rotation.z,
            robotXform.rotation.w,
            robotVform.linear.x, # (m/s)
            robotVform.linear.y,
            robotVform.linear.z,
            robotVform.angular.x, # (rad/s)
            robotVform.angular.y,
            robotVform.angular.z
        )
        sServiceIn.addRespPacket(retXformPkt, websocket)
        self.controlState.setXform(robotXform.position.x, robotXform.position.y, robotXform.position.z, robotXform.rotation.x, robotXform.rotation.y, robotXform.rotation.z, robotXform.rotation.w)

        return None

    def retrieveRobotState(self, sServiceIn, websocket):
        stateDict = {}        
        robotState = self._robot_state_client.get_robot_state()

        # Encode general info
        battery_state = robotState.battery_states[0]
        if battery_state.charge_percentage.value:
            bar_len = battery_state.charge_percentage.value
            stateDict["battery_percentage"] = bar_len / 100.0
        else:
            stateDict["battery_percentage"] = 0

        stateDict["battery_timeleft"] = ""
        if battery_state.estimated_runtime:
            time_left = ' ({})'.format(secs_to_hms(battery_state.estimated_runtime.seconds))
            stateDict["battery_timeleft"] = time_left

        stateDict["cameraQuality"] = int(self.cameraQuality)
        
        if(self.controlStateRecordFlag == True):
            stateDict["controlStateRecord"] = int(self.controlStateRecordFlag)

        stateDict["spotMode"] = self._spotMode
        stateDict["sCR"] = int(self.spotCamIsRecording())
        stateDict["imu"] = int(self.alwaysPublishOdom)

        jsonStr = json.dumps(stateDict)
        #print(jsonStr)

        retStatusPkt = spotMsgs.StatusCommandPacket()
        retStatusPkt.setMsg(jsonStr)

        return retStatusPkt
    
    def sendBackRobotLogMsg(self, msgIn, sServiceIn, websocket):
        stateDict = {}
        stateDict["logMsg"] = msgIn
        jsonStr = json.dumps(stateDict)

        retStatusPkt = spotMsgs.StatusCommandPacket()
        retStatusPkt.setMsg(jsonStr)
        sServiceIn.addRespPacket(retStatusPkt, websocket)

    def _try_grpc(self, desc, thunk):
        try:
            return thunk()
        except (ResponseError, RpcError) as err:
            print("Failed {}: {}".format(desc, err))
            return None        

    def _start_robot_command(self, desc, command_proto, end_time_secs=None):
        def _start_command():
            self._robot_command_client.robot_command(lease=None, command=command_proto,
                                                        end_time_secs=end_time_secs)
        

        self._try_grpc(desc, _start_command)


    def _velocity_cmd_helper(self, desc='', v_x=0.0, v_y=0.0, v_rot=0.0):
        self._start_robot_command(desc,
                                  RobotCommandBuilder.velocity_command(
                                      v_x=v_x, v_y=v_y, v_rot=v_rot, params=self.mobilityParams),
                                  end_time_secs=time.time() + VELOCITY_CMD_DURATION)       
             
    def runMoveVel(self, cmdPacket):
        if self.robotIsOn == False:
            return

        self._velocity_cmd_helper(cmdPacket.descrip, cmdPacket.vX, cmdPacket.vY, cmdPacket.vRot)
        self.controlState.setMoveVel(cmdPacket.vX, cmdPacket.vY, cmdPacket.vRot)

    def relayRespPacket(self, cmdPacket, sServiceIn, websocket):
        sServiceIn.addRespPacket(cmdPacket, websocket)
        return None
    
    def runRobotCommand(self, cmdPacket):
        print("Robot command %s" %cmdPacket.msg)
        if cmdPacket.msg == "self_right":
            self._start_robot_command(cmdPacket.msg, RobotCommandBuilder.selfright_command())
        elif cmdPacket.msg == "self_left":
            self._start_robot_command(cmdPacket.msg, RobotCommandBuilder.selfleft_command())
        elif cmdPacket.msg == "sit":
            self._start_robot_command(cmdPacket.msg, RobotCommandBuilder.sit_command())
        elif cmdPacket.msg == "stand":
            self._start_robot_command(cmdPacket.msg, RobotCommandBuilder.stand_command(params=self.mobilityParams))

    def powerRobot(self, flagIn):
        if (flagIn == 1) and (self.robot.is_powered_on() == False):
            # Now, we are ready to power on the robot. This call will block until the power
            # is on. Commands would fail if this did not happen. We can also check that the robot is
            # powered at any point.
            print("Powering ON")
            self.robot.power_on(timeout_sec=20)
            assert self.robot.is_powered_on(), "Robot power on failed."
            self.robotIsOn = True
        elif (flagIn == 0) and (self.robot.is_powered_on() == True):
            # Power the robot off. By specifying "cut_immediately=False", a safe power off command
            # is issued to the robot. This will attempt to sit the robot before powering off.
            print("Powering OFF")
            self.robot.power_off(cut_immediately=False, timeout_sec=20)
            self.robotIsOn = False

    def spotCamSendStream(self, websocket):
        if self.spotCamStream == None:
            print("No SpotCam/SpotCamStream to send back")
            return None
        
        sImg = self.spotCamStream.getImgStreamBytes(self.cameraQuality)
        if sImg == None:
            return None
        
        if self.controlState.cameraSpotCamActive:
            self.controlState.setCameraSpotCam(sImg[1].tobytes())
        
        sendImgPairs = [(sImg[1].tobytes(), "SpotCam")]
        imgListPkt = spotMsgs.ImageListCommandPacket.createImgListPacket(sendImgPairs)
        return imgListPkt

    def _sourceCameraNames(self):
        if len(self.camSourceNames) == 0:
            allSources = self.image_client.list_image_sources()
            self.camSourceNames = [source.name for source in allSources]
            print("There are %d cameras:\n %s" %(len(self.camSourceNames), str(allSources)))
        return self.camSourceNames

    def _imagesReceivedCB(self, _fut):
        m = 0
        sendImgPairs = []
        image_responses = _fut.result()
        for img in image_responses:
            img_data = img.shot.image.data
            sendImgPairs.append((img_data, self.retrieveImageNames[m]))
            m = m + 1
        
        imgListPkt = spotMsgs.ImageListCommandPacket.createImgListPacket(sendImgPairs)
        with self.msgLock:
            for cWebsocket in self.msgWebsockets:
                self._activeService.addRespPacket(imgListPkt, cWebsocket)
            self.msgWebsockets.clear()                

    def captureImages(self, sServiceIn, websocket, subNamesList):
        #if self.robotIsOn == False:
        #    return

        # Spot has five sensors around the body. Each sensor consists of a stereo pair and a
        # fisheye camera. The list_image_sources RPC gives a list of image sources which are
        # available to the API client. Images are captured via calls to the get_image RPC.
        # Images can be requested from multiple image sources in one call.
        # Available Images: ['back_fisheye_image', 'frontleft_fisheye_image', 'frontright_fisheye_image',
        # 'left_fisheye_image', 'right_fisheye_image']
        sourceNameList = self._sourceCameraNames()
        # Async version
        self._activeService = sServiceIn

        with self.msgLock:
            self.msgWebsockets.append(websocket)

        subImageList = sourceNameList
        if len(subImageList) > 0:
            subImageList = subNamesList

        if self.captureDepth:
            # Add in depth cameras
            '''
            depthCamList = []
            for cName in subImageList:
                depthCamList.append(self.getDepthCamName(cName))
            subImageList.extend(depthCamList)
            '''

        self.retrieveImageNames = subImageList
        #Request the images asyncronously, we can use the default method: images_future = self.image_client.get_image_from_sources_async(subImageList)
        # Or a more customized form below:
        images_req = [bosdyn.client.image.build_image_request(img_src, quality_percent=self.cameraQuality) for img_src in subImageList]
        images_future = self.image_client.get_image_async(images_req)
        images_future.add_done_callback(self._imagesReceivedCB)
        return None
        
        '''
        # Non-Async version        
        #image_responses = self.image_client.get_image_from_sources(sourceNameList)

        subImageList = sourceNameList
        if len(subImageList) > 0:
            subImageList = subNamesList

        image_responses = self.image_client.get_image_from_sources(subImageList)
        m = 0
        sendImgPairs = []
        for img in image_responses:
            img_data = img.shot.image.data
            sendImgPairs.append((img_data, subImageList[m]))
            m = m + 1

        # Send back image list
        return spotMsgs.ImageListCommandPacket.createImgListPacket(sendImgPairs)
        '''