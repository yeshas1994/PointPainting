import os
import sys
import asyncio
import websockets
import logging
import msgpack
import spotSDKLayer
from random import randrange
import spotMsgs
import threading
import time
import random
import json
import requests
import numpy as np
import ssl
import traceback

# Test OpenCV
import cv2
cap = cv2.VideoCapture('./vidTest.mp4')

globalStreamCnt = 0
def getImgPacket():
    global globalStreamCnt

    sendImgPairs = []
    globalStreamCnt += 1
    sourceNameList = ['back_fisheye_image', 'frontleft_fisheye_image', 'frontright_fisheye_image',
        'left_fisheye_image', 'right_fisheye_image']    
    baseFilenames = ["./person.jpg", "./person2.jpg", "./person3.jpg", "./person4.jpg", "./bird1.jpg"]
    random.shuffle(baseFilenames)

    for i in range(0, 1):
        curName =  sourceNameList[i]
        if i == 0 and cap.isOpened():
            ret, frame = cap.read()
            if ret == False:
                cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                ret, frame = cap.read()

            frame = cv2.resize(frame,(640, 480))
            grayImage = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 100]
            result, encimg = cv2.imencode('.jpg', grayImage, encode_param)
            sendImgPairs.append((encimg.tobytes(),curName))
            continue

        testFilename = baseFilenames[i]

        with open(testFilename, mode="rb") as fp:
            fileContent = fp.read()
            sendImgPairs.append((fileContent, curName))

    #respPacket = spotMsgs.ImageCommandPacket.createImgPacket(fileContent)
    respPacket = spotMsgs.ImageListCommandPacket.createImgListPacket(sendImgPairs)
    return respPacket

# SpotConnection
class SpotConnection(object):
    def __init__(self, websocket):
        self.websocket = websocket
        self.priority = 100
        self.canMove = True
        self.alwaysReceive = False

    @staticmethod
    def isMovementType(typeIn):
        return (typeIn == spotMsgs.COMMAND_MOVE_VEL_TYPE) or (typeIn == spotMsgs.COMMAND_ROBOT_CMD_TYPE) or (typeIn == spotMsgs.COMMAND_MOTION_PARAMS_TYPE)

    def filterPkt(self, pktIn):
        ''' If returns True, then you can continue to process the packet for the Robot '''
        pID = pktIn.getPacketTypeID()
        if(SpotConnection.isMovementType(pID)):
            return self.canMove

        if pID != spotMsgs.COMMAND_CONTROL_PERMISSIONS_TYPE:
            return True
        
        cData = pktIn.getDataMap()
        if "priority" in cData:
            self.priority = int(cData["priority"])

        if "canMove" in cData:
            self.canMove = bool(cData["canMove"])

        if "alwaysReceive" in cData:
            self.alwaysReceive = bool(cData["alwaysReceive"])

        # Is of Permissions Type so return False ( the caller should skip any further processing )
        return False

# SpotService
class SpotService(object):
    def __init__(self, appTokenPath, hostname, username, password, cameraQuality, registerServices, isTesting):
        if isTesting:
            self.mainSpotRobot = None
        else:
            self.mainSpotRobot = self.startSpotRobot(
                appTokenPath, 
                hostname, 
                username, 
                password, 
                cameraQuality, 
                True)
        self.mainSpotRobotOptions = (appTokenPath, hostname, username, password, cameraQuality, registerServices)
        self.mainSpotCamOptions = None

        self.connections = {}
        self.respPackets = []
        self.respTask = None
        self.respLock = threading.Lock()
        self.tickLock = threading.Lock()
        self.monitorServers = [] # For sending monitoring of image streams
        self.slamVideoFilename = ""
        self.slamImagesFolder = ""
        self.slamVideoUseImages = False
        self.slamCameraName = "frontleft_fisheye_image"
        self.slamVideoNumFrames = 0
        self.slamVideoMaxNumFrames = 30
        self.fourcc = None
        self.videoOut = None
        self.shutdown = False
    
    def startSpotCam(self, folderIn, hostname):
        self.mainSpotCamOptions = [folderIn, hostname]
        self.restoreOrStartSpotCam()
    
    def restoreOrStartSpotCam(self):
        if self.mainSpotCamOptions == None:
            return
        self.mainSpotRobot.spotCamStart(self.mainSpotCamOptions[0], self.mainSpotCamOptions[1])

    def startSpotRobot(self, appTokenPath, hostname, username, password, cameraQuality, registerServices, keepTrying=True):
        '''Will attempt to keep trying to connect to the Spot Robot until success'''
        while True:
            try:
                mainSpotRobot = spotSDKLayer.SpotRobot(appTokenPath, hostname, username, password, cameraQuality, registerServices)
                print("Successfully connected to Spot.")
                return mainSpotRobot
            except:
                traceback.print_exc()
                if keepTrying == False:
                    break
                print("Failed to connect to Spot. Will try again in 5 secs...")
                time.sleep(5)   
        return None        
        
    def createVideoWriter(self):
        self.fourcc = cv2.VideoWriter_fourcc(*'MP4V')
        self.videoOut = cv2.VideoWriter(self.slamVideoFilename, self.fourcc, 15.0, (640,480))
        print("Recording SLAM Video into %s" %self.slamVideoFilename)
        self.videoFrame = 0
        if self.mainSpotRobot:
            self.mainSpotRobot.captureDepth = True

    def stopVideoWriter(self):
        if(self.videoOut):
            self.videoOut.release()
            print("Finished Recording SLAM Video for: %s" %self.slamCameraName)
            self.videoOut = None

        if self.mainSpotRobot:
            self.mainSpotRobot.captureDepth = False

    def addConnection(self, websocket):
        self.connections[websocket] = SpotConnection(websocket)

    def removeConnection(self, websocket):
        del self.connections[websocket]

    def processFilterConnections(self, websocket, pktIn):
        ''' Process all connections to decide whether you can use the packet for Robot control or not '''
        cConnection = self.connections[websocket]
        canUse = cConnection.filterPkt(pktIn)
        return canUse

    def addRespPacket(self, packetIn, websocket):
        with self.respLock:
            self.respPackets.append((packetIn, websocket))
    
    async def processRespPacket(self):
        try:
            while True:
                await asyncio.sleep(0.05)
                with self.respLock:
                    for cPair in self.respPackets:
                        cPacket = cPair[0]
                        cWebSocket = cPair[1]
                        sendBytes = cPacket.encode()
                        self.pushToMonitorServers(cPacket, sendBytes)
                        self.writeVideo(cPacket, sendBytes)
                        await cWebSocket.send(sendBytes)
                        
                        for cKey in self.connections.keys():
                            cConnection = self.connections[cKey]
                            if cConnection.alwaysReceive and cConnection.websocket != cWebSocket:
                                await cConnection.websocket.send(sendBytes)

                    self.respPackets.clear()
        except:
            traceback.print_exc()
            self.respPackets.clear()

    def pushToMonitorServers(self, pktIn, bytesIn):
        if(len(self.monitorServers) == 0):
            return

        if isinstance(pktIn, spotMsgs.ImageListCommandPacket):
            for pushAddr in self.monitorServers:
                requests.post(url=pushAddr, data=bytesIn, headers={'Content-Type': 'application/octet-stream'}) 

    def writeVideo(self, pktIn, bytesIn):
        if(self.videoOut == None):
            return

        if isinstance(pktIn, spotMsgs.ImageListCommandPacket):
            for imgPair in pktIn.imageList:
                imgData = imgPair[0]
                imgName = imgPair[1]
                if imgName == self.slamCameraName:
                    image = np.asarray(bytearray(imgData),  dtype="uint8")
                    image = cv2.imdecode(image, cv2.IMREAD_COLOR)

                    if self.slamVideoUseImages:
                        frameStr = "%06d" % self.videoFrame
                        subFolderStr = "/color/"
                        wExtension = ".jpg"
                        if self.mainSpotRobot.isDepthCamName(imgName):
                            subFolderStr = "/depth/"
                            wExtension = ".png"

                        wFolder = self.slamImagesFolder + subFolderStr
                        if os.path.exists(wFolder) == False:
                            os.mkdir(wFolder)

                        wFilename = wFolder + "slam_" + frameStr + wExtension
                        cv2.imwrite(wFilename, image)
                    
                    self.videoOut.write(image)

            self.videoFrame += 1

    def processNonSpotPacket(self, cmdPacket):
        if isinstance(cmdPacket, spotMsgs.MsgStrCommandPacket):
            if(cmdPacket.msg == "VIDEO_RECORD_START"):
                self.createVideoWriter()
                return True
            elif(cmdPacket.msg == "VIDEO_RECORD_STOP"):
                self.stopVideoWriter()
                return True
            elif(cmdPacket.msg == "RESTART_SPOT_SERVER"):
                print("Restarting Spot Server...")
                appTokenPath, hostname, username, password, cameraQuality, registerServices = self.mainSpotRobotOptions
                self.mainSpotRobot.shutdown(True)
                self.mainSpotRobot = self.startSpotRobot(appTokenPath, hostname, username, password, cameraQuality, registerServices, True)
                self.restoreOrStartSpotCam()
            elif(cmdPacket.msg == "TERMINATE_CONNECTION"):
                self.shutdown = True
                if self.mainSpotRobot:
                    self.mainSpotRobot.shutdown(True)
                    self.mainSpotRobot = None

        return False

    async def processTestTick(self, cmdPacket, websocket, path):
        if isinstance(cmdPacket, spotMsgs.ImageStreamTogglePacket):
            testImgsPacket = getImgPacket()
            sendBytes = testImgsPacket.encode()
            self.pushToMonitorServers(testImgsPacket, sendBytes)
            self.addRespPacket(testImgsPacket, websocket)
        elif isinstance(cmdPacket, spotMsgs.SetWaypointsPacket) or isinstance(cmdPacket, spotMsgs.MoveToWaypointsPacket):
            self.addRespPacket(cmdPacket, websocket)
    
    async def processRobotTick(self, cmdPacket, websocket, path):
        retData = self.mainSpotRobot.tick(cmdPacket, self, websocket)
        if retData != None:
            if isinstance(retData, spotMsgs.BaseCommandPacket):
                # Send single data response
                await websocket.send(retData.encode())
            elif isinstance(retData, list):
                # A list of packets to possibly send
                for subData in retData:
                    # Send sub data response
                    if isinstance(subData, spotMsgs.BaseCommandPacket):
                        await websocket.send(subData.encode())


    async def processRobot(self, websocket, path):
        # Main routine to perform robot processing
        while not self.shutdown:
            dataBytesIn = await websocket.recv()
            cmdPacket = spotMsgs.processCommandPacketFromBytes(dataBytesIn)

            with self.tickLock:
                canSkipTick = self.processNonSpotPacket(cmdPacket)
                if(canSkipTick == False):
                    if self.processFilterConnections(websocket, cmdPacket):
                        if self.mainSpotRobot:
                            # Tick the Robot
                            await self.processRobotTick(cmdPacket, websocket, path)
                        else:
                            # Test Server case, send back test images if required
                            await self.processTestTick(cmdPacket, websocket, path)

    async def run(self, websocket, path):
        print("A new client connected with path: %s" %path)
        self.addConnection(websocket)
        print("Server has total of %d connected clients." %len(self.connections))

        robotTask = asyncio.create_task(self.processRobot(websocket, path))        
        if (self.respTask == None) or (len(self.connections) == 1):
            print("Creating Response Packet Loop...")
            self.respTask = asyncio.create_task(self.processRespPacket())

        try:
            await robotTask
        except:
            traceback.print_exc()
            print("Client exited abnormally. Removing client: %s" %path)

        self.removeConnection(websocket)

mainService = None
async def start(websocket, path):
    global mainService
    await mainService.run(websocket, path)

def startWebSocketsFunc():
    # Or Maybe start the SpotServer here? Not sure if it plays well with async io?
    # WIFI: SNPS-LIMA#005, PWD: GovTechLim@#005
    # user5, PWD: GovTechLim@005

    # There MUST be a config file present to load in the user parameters.
    # It is called spotServerConfig.json, in the same directory as the server
    configFile = open('./spotServerConfig.json')
    configJSON = json.load(configFile)

    useTLS = False
    pemFile = ""
    ssl_context = None
    if ("useTLS" in configJSON) and ("pemFile" in configJSON):
        useTLS = configJSON["useTLS"]
        if useTLS:
            print("Using TLS Secure Transport")
            pemFile = configJSON["pemFile"]
            ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
            ssl_context.load_cert_chain(pemFile)

    appTokenPath = configJSON['appTokenPath'] #"../../sggovtech.token"
    hostname = configJSON['hostname'] #"192.168.80.3"
    username = configJSON['username'] #"user5"
    password = configJSON['password'] #"GovTechLim@#005"
    isTesting = bool(configJSON['test'])
    print("Trying to connect to robot at: %s with user: %s" %(hostname, username))

    cameraQuality = 75
    if(isTesting):
        print("This is a test server, no actual connection to the robot will be made.")
    else:
        if "cameraQuality" in configJSON:
            cameraQuality = configJSON["cameraQuality"]
            print("The robot camera quality is set to: %d" %cameraQuality)

    global mainService
    registerServices = False
    if("spotCamFolder" in configJSON and "spotCamActive" in configJSON):
        registerServices = configJSON["spotCamActive"]

    mainService = SpotService(appTokenPath, hostname, username, password, cameraQuality, registerServices, isTesting)
    mainService.monitorServers = configJSON['monitorServers']
    print("There are the following monitor servers set: %s" %(mainService.monitorServers))

    if(("slamVideoFilename" in configJSON) and ("slamCameraName" in configJSON)):
        mainService.slamVideoFilename = configJSON["slamVideoFilename"]
        mainService.slamCameraName = configJSON["slamCameraName"]
        mainService.slamVideoUseImages = configJSON["slamVideoUseImages"]
        mainService.slamImagesFolder = configJSON["slamImagesFolder"]

    if("spotControlStateRunFolder" in configJSON and (mainService.mainSpotRobot != None)):
        mainService.mainSpotRobot.controlStateRecordFolder = configJSON["spotControlStateRunFolder"]
        print("Setting Control State Record folder to %s" %mainService.mainSpotRobot.controlStateRecordFolder)

    if("alwaysPublishOdom" in configJSON and (mainService.mainSpotRobot != None)):
        mainService.mainSpotRobot.alwaysPublishOdom = configJSON["alwaysPublishOdom"]
        print("Setting always publish Odom to %d" %int(mainService.mainSpotRobot.alwaysPublishOdom))

    if ("navGraphFolder" in configJSON and (mainService.mainSpotRobot != None)):
        mainService.mainSpotRobot.setNavRecordFolder(configJSON["navGraphFolder"])
        print("Setting up NavGraph with folder %s" %configJSON["navGraphFolder"])
    
    if ("navMode" in configJSON and (mainService.mainSpotRobot != None)):
        mainService.mainSpotRobot.setNavRunMode(configJSON["navMode"])

    if("spotCamFolder" in configJSON and "spotCamActive" in configJSON and (mainService.mainSpotRobot != None)):
        if configJSON["spotCamActive"] == True:
            sHostname = "192.168.80.3"
            if "spotCamAddress" in configJSON:
                sHostname = configJSON["spotCamAddress"]
            mainService.startSpotCam(configJSON["spotCamFolder"], sHostname)

    serverAddress = configJSON['serverAddress'] #"localhost"
    serverPort = int(configJSON['port']) #8765    
    print("Starting Spot Server with address: %s and port: %d" %(serverAddress, serverPort))
    server = websockets.serve(start, serverAddress, serverPort, ssl=ssl_context)

    asyncio.get_event_loop().run_until_complete(server)
    asyncio.get_event_loop().run_forever()

if __name__ == "__main__":
    startWebSocketsFunc()

