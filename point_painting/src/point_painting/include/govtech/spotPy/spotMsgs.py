import os
import sys
import asyncio
import websockets
import logging
import msgpack
import json

# Command Type Definitions
COMMAND_MSG_TYPE = 1
COMMAND_BYTES_TYPE = 2
COMMAND_MOVE_VEL_TYPE = 3
COMMAND_POWER_ROBOT_TYPE = 4
COMMAND_ROBOT_CMD_TYPE = 5
COMMAND_IMG_TYPE = 6
COMMAND_IMG_STREAM_TOGGLE_TYPE = 7
COMMAND_IMG_LIST_TYPE = 8
COMMAND_STATUS_TYPE = 9
COMMAND_MOTION_PARAMS_TYPE = 10
COMMAND_SETTINGS_TYPE = 11
COMMAND_BYTES_LIST_TYPE = 12
COMMAND_XFORM_TYPE = 13
COMMAND_CONTROL_PERMISSIONS_TYPE = 14
COMMAND_WAYPOINTS_TYPE = 15
COMMAND_MOVE_WAYPOINT_TYPE = 16
COMMAND_SPOT_NAVGRAPH_TYPE = 17
COMMAND_IMG_LIST_WITH_META_TYPE = 18
COMMAND_UPDATE_POSE_TYPE = 19
COMMAND_INITIAL_POSITION_TYPE = 20

# Command Type Factor Map
MsgFactoryMap = {}
def RegisterCommandFactoryObj(typeIn, createFn):
    global MsgFactoryMap
    if(typeIn in MsgFactoryMap):
        raise Exception("DOS Error! Cannot register type with id: %d and function %s" %(typeIn, createFn))

    MsgFactoryMap[typeIn] = createFn
    print("Registering DOS Command Packet Msg with id: %d and function: %s" %(typeIn, createFn))

# Command Types
class BaseCommandPacket:
    def __init__(self, dataArrayIn = None):
        if(dataArrayIn != None):
            self.parseDataArray(dataArrayIn)

    def encode(self):
        newArray = []
        newArray.append(self.getPacketTypeID())
        self.encodeToArray(newArray)
        packData = msgpack.packb(newArray, use_bin_type=True)
        return packData

    '''Override to return packet type'''
    def getPacketTypeID(self):
        return -1
        
    '''Override to append elements to the array for encoding'''
    def encodeToArray(self, arrayIn):
        pass
    
    '''Override to parse array into class'''
    def parseDataArray(self, arrayIn):
        pass

class MsgStrCommandPacket(BaseCommandPacket):
    def __init__(self, dataArrayIn = None):
        super().__init__(dataArrayIn)

    def setMsg(self, msgIn):
        self.msg = msgIn

    def getPacketTypeID(self):
        return COMMAND_MSG_TYPE
        
    def encodeToArray(self, arrayIn):
        arrayIn.append(self.msg)
    
    def parseDataArray(self, arrayIn):
        self.msg = arrayIn[1]

    @staticmethod
    def createSelf(arrayIn):
        return MsgStrCommandPacket(arrayIn)

RegisterCommandFactoryObj(COMMAND_MSG_TYPE, MsgStrCommandPacket.createSelf)        

class StatusCommandPacket(MsgStrCommandPacket):
    def __init__(self, dataArrayIn = None):
        super().__init__(dataArrayIn)    

    def getPacketTypeID(self):
        return COMMAND_STATUS_TYPE

    @staticmethod
    def createSelf(arrayIn):
        return StatusCommandPacket(arrayIn)

RegisterCommandFactoryObj(COMMAND_STATUS_TYPE, StatusCommandPacket.createSelf)        

class SettingsCommandPacket(MsgStrCommandPacket):
    def __init__(self, dataArrayIn = None):
        super().__init__(dataArrayIn)    

    def getPacketTypeID(self):
        return COMMAND_SETTINGS_TYPE

    def getDataMap(self):
        return json.loads(self.msg)

    @staticmethod
    def createSelf(arrayIn):
        return SettingsCommandPacket(arrayIn)

RegisterCommandFactoryObj(COMMAND_SETTINGS_TYPE, SettingsCommandPacket.createSelf)

class ControlPermissionsPacket(MsgStrCommandPacket):
    def __init__(self, dataArrayIn = None):
        super().__init__(dataArrayIn)    

    def getPacketTypeID(self):
        return COMMAND_CONTROL_PERMISSIONS_TYPE

    def getDataMap(self):
        return json.loads(self.msg)

    @staticmethod
    def createSelf(arrayIn):
        return ControlPermissionsPacket(arrayIn)

RegisterCommandFactoryObj(COMMAND_CONTROL_PERMISSIONS_TYPE, ControlPermissionsPacket.createSelf)

class SpotGraphNavCommandPacket(MsgStrCommandPacket):
    def __init__(self, dataArrayIn = None):
        super().__init__(dataArrayIn)    

    def getPacketTypeID(self):
        return COMMAND_SPOT_NAVGRAPH_TYPE

    def getDataMap(self):
        return json.loads(self.msg)

    @staticmethod
    def createSelf(arrayIn):
        return SpotGraphNavCommandPacket(arrayIn)

RegisterCommandFactoryObj(COMMAND_SPOT_NAVGRAPH_TYPE, SpotGraphNavCommandPacket.createSelf)

class MotionParamsCommandPacket(MsgStrCommandPacket):
    def __init__(self, dataArrayIn = None):
        super().__init__(dataArrayIn)    

    def getPacketTypeID(self):
        return COMMAND_MOTION_PARAMS_TYPE
    
    def getDataMap(self):
        return json.loads(self.msg)

    @staticmethod
    def createSelf(arrayIn):
        return MotionParamsCommandPacket(arrayIn)

RegisterCommandFactoryObj(COMMAND_MOTION_PARAMS_TYPE, MotionParamsCommandPacket.createSelf)        

class BytesCommandPacket(BaseCommandPacket):
    def __init__(self, dataArrayIn = None):
        self.bytes = bytearray("", 'utf-8')
        super().__init__(dataArrayIn)

    def getPacketTypeID(self):
        return COMMAND_BYTES_TYPE
        
    def encodeToArray(self, arrayIn):
        arrayIn.append(self.bytes)
    
    def parseDataArray(arrayIn):
        self.bytes = arrayIn[1]

    @staticmethod
    def createSelf(arrayIn):
        return BytesCommandPacket(arrayIn)

RegisterCommandFactoryObj(COMMAND_BYTES_TYPE, BytesCommandPacket.createSelf)      

class BytesListCommandPacket(BaseCommandPacket):
    def __init__(self, dataArrayIn = None):
        self.blobList = []
        super().__init__(dataArrayIn)

    def getPacketTypeID(self):
        return COMMAND_BYTES_LIST_TYPE        

    def encodeToArray(self, arrayIn):
        arrayIn.append(len(self.blobList))

        for cBlob in self.blobList:
            # [(bytes, label) ...]
            cBytes, cSubType, cLabel = cBlob
            arrayIn.append(cBytes)
            arrayIn.append(cSubType)
            arrayIn.append(cLabel)
    
    def parseDataArray(self, arrayIn):
        numBlobs = arrayIn[1]
        for i in range(0, numBlobs):
            baseIdx = 2 + (i * 3)
            self.blobList.append((arrayIn[baseIdx], arrayIn[baseIdx + 1], arrayIn[baseIdx + 2]))

    @staticmethod
    def createSelf(arrayIn):
        return BytesListCommandPacket(arrayIn)                  

    @staticmethod
    def createImgListPacket(blobTupleListIn):
        '''blobTupleListIn is in format: [(bytes ( char array), subType (uint8), label (string)) ...]'''
        retPkt = BytesListCommandPacket()
        retPkt.blobList = blobTupleListIn
        return retPkt

RegisterCommandFactoryObj(COMMAND_BYTES_LIST_TYPE, BytesListCommandPacket.createSelf)

class XformCommandPacket(BaseCommandPacket):
    def __init__(self, dataArrayIn = None):
        self.subType = 0
        # Position Orientation
        self.pX = 0
        self.pY = 0
        self.pZ = 0

        self.qX = 0
        self.qY = 0
        self.qZ = 0
        self.qW = 0

        # Velocities
        self.lVelX = 0
        self.lVelY = 0
        self.lVelZ = 0

        self.aVelX = 0
        self.aVelY = 0
        self.aVelZ = 0

        # Accelerations
        self.lAccX = 0
        self.lAccY = 0
        self.lAccZ = 0

        self.aAccX = 0
        self.aAccY = 0
        self.aAccZ = 0

        super().__init__(dataArrayIn)

    def getPacketTypeID(self):
        return COMMAND_XFORM_TYPE        

    def encodeToArray(self, arrayIn):
        arrayIn.append(self.subType)

        arrayIn.append(self.pX)
        arrayIn.append(self.pY)
        arrayIn.append(self.pZ)

        arrayIn.append(self.qX)
        arrayIn.append(self.qY)
        arrayIn.append(self.qZ)
        arrayIn.append(self.qW)

        arrayIn.append(self.lVelX)
        arrayIn.append(self.lVelY)
        arrayIn.append(self.lVelZ)

        arrayIn.append(self.aVelX)
        arrayIn.append(self.aVelY)
        arrayIn.append(self.aVelZ)

        arrayIn.append(self.lAccX)
        arrayIn.append(self.lAccY)
        arrayIn.append(self.lAccZ)

        arrayIn.append(self.aAccX)
        arrayIn.append(self.aAccY)
        arrayIn.append(self.aAccZ)

    def parseDataArray(self, arrayIn):
        self.subType = arrayIn[1]

        self.pX = arrayIn[2]
        self.pY = arrayIn[3]
        self.pZ = arrayIn[4]

        self.qX = arrayIn[5]
        self.qY = arrayIn[6]
        self.qZ = arrayIn[7]
        self.qW = arrayIn[8]

        self.lVelX = arrayIn[9]
        self.lVelY = arrayIn[10]
        self.lVelZ = arrayIn[11]

        self.aVelX = arrayIn[12]
        self.aVelY = arrayIn[13]
        self.aVelZ = arrayIn[14]

        self.lAccX = arrayIn[15]
        self.lAccY = arrayIn[16]
        self.lAccZ = arrayIn[17]

        self.aAccX = arrayIn[18]
        self.aAccY = arrayIn[19]
        self.aAccZ = arrayIn[20]

    @staticmethod
    def createSelf(arrayIn):
        return XformCommandPacket(arrayIn)                  

    @staticmethod
    def createXformPacket(subType, 
        pX, pY, pZ, qX, qY, qZ, qW, 
        lVelX=0, lVelY=0, lVelZ=0, aVelX=0, aVelY=0, aVelZ=0,
        lAccX=0, lAccY=0, lAccZ=0, aAccX=0, aAccY=0, aAccZ=0):
        retPkt = XformCommandPacket()
        retPkt.subType = subType

        retPkt.pX = pX
        retPkt.pY = pY
        retPkt.pZ = pZ

        retPkt.qX = qX
        retPkt.qY = qY
        retPkt.qZ = qZ
        retPkt.qW = qW

        retPkt.lVelX = lVelX
        retPkt.lVelY = lVelY
        retPkt.lVelZ = lVelZ

        retPkt.aVelX = aVelX
        retPkt.aVelY = aVelY
        retPkt.aVelZ = aVelZ

        retPkt.lAccX = lAccX
        retPkt.lAccY = lAccY
        retPkt.lAccZ = lAccZ

        retPkt.aAccX = aAccX
        retPkt.aAccY = aAccY
        retPkt.aAccZ = aAccZ

        return retPkt

RegisterCommandFactoryObj(COMMAND_XFORM_TYPE, XformCommandPacket.createSelf) 

class PowerRobotCommandPacket(BaseCommandPacket):
    def __init__(self, dataArrayIn = None):
        self.state = 0
        super().__init__(dataArrayIn)

    def getPacketTypeID(self):
        return COMMAND_POWER_ROBOT_TYPE
        
    def encodeToArray(self, arrayIn):
        arrayIn.append(self.state)
    
    def parseDataArray(self, arrayIn):
        self.state = arrayIn[1]

    @staticmethod
    def createSelf(arrayIn):
        return PowerRobotCommandPacket(arrayIn)

RegisterCommandFactoryObj(COMMAND_POWER_ROBOT_TYPE, PowerRobotCommandPacket.createSelf)        

class RobotCommandPacket(MsgStrCommandPacket):
    def __init__(self, dataArrayIn = None):
        self.parseDataArray(dataArrayIn)

    def getPacketTypeID(self):
        return COMMAND_ROBOT_CMD_TYPE        

    @staticmethod
    def createSelf(arrayIn):
        return RobotCommandPacket(arrayIn)        

RegisterCommandFactoryObj(COMMAND_ROBOT_CMD_TYPE, RobotCommandPacket.createSelf)        

class MoveVelCommandPacket(BaseCommandPacket):
    def __init__(self, dataArrayIn = None):
        self.descrip = ""
        self.vX = 0
        self.vY = 0
        self.vRot = 0
        super().__init__(dataArrayIn)

    def getPacketTypeID(self):
        return COMMAND_MOVE_VEL_TYPE
        
    def encodeToArray(self, arrayIn):
        arrayIn.append(self.descrip)
        arrayIn.append(self.vX)
        arrayIn.append(self.vY)
        arrayIn.append(self.vZ)
        arrayIn.append(self.vRot)
    
    def parseDataArray(self, arrayIn):
        self.descrip = arrayIn[1]
        self.vX = arrayIn[2]
        self.vY = arrayIn[3]
        self.vRot = arrayIn[4]

    @staticmethod
    def createSelf(arrayIn):
        return MoveVelCommandPacket(arrayIn)               

RegisterCommandFactoryObj(COMMAND_MOVE_VEL_TYPE, MoveVelCommandPacket.createSelf)       

class SetWaypointsPacket(BaseCommandPacket):
    def __init__(self, dataArrayIn = None):
        self.num = 0
        self.floatArray = []
        super().__init__(dataArrayIn)

    def getPacketTypeID(self):
        return COMMAND_WAYPOINTS_TYPE
        
    def encodeToArray(self, arrayIn):
        arrayIn.append(self.num)
        for i in range(0, self.num):
            baseIdx = i * 2
            arrayIn.append(self.floatArray[baseIdx])
            arrayIn.append(self.floatArray[baseIdx+1])
    
    def parseDataArray(self, arrayIn):
        self.num = arrayIn[1]
        for i in range(0, self.num):
            baseIdx = 2 + (i * 2)
            self.floatArray.append(arrayIn[baseIdx])
            self.floatArray.append(arrayIn[baseIdx+1])

    @staticmethod
    def createSelf(arrayIn):
        return SetWaypointsPacket(arrayIn)

RegisterCommandFactoryObj(COMMAND_WAYPOINTS_TYPE, SetWaypointsPacket.createSelf)  

class MoveToWaypointsPacket(BaseCommandPacket):
    def __init__(self, dataArrayIn = None):
        self.index = 0
        super().__init__(dataArrayIn)

    def getPacketTypeID(self):
        return COMMAND_MOVE_WAYPOINT_TYPE
        
    def encodeToArray(self, arrayIn):
        arrayIn.append(self.index)
    
    def parseDataArray(self, arrayIn):
        self.index = arrayIn[1]

    @staticmethod
    def createSelf(arrayIn):
        return MoveToWaypointsPacket(arrayIn)

RegisterCommandFactoryObj(COMMAND_MOVE_WAYPOINT_TYPE, MoveToWaypointsPacket.createSelf)  

class UpdatePosePacket(BaseCommandPacket):
    def __init__(self, dataArrayIn = None):
        self.pX = 0
        self.pY = 0
        self.pZ = 0
        self.qX = 0
        self.qY = 0
        self.qZ = 0
        self.qW = 0
        super().__init__(dataArrayIn)

    def getPacketTypeID(self):
        return COMMAND_UPDATE_POSE_TYPE
        
    def encodeToArray(self, arrayIn):
        arrayIn.append(self.pX)
        arrayIn.append(self.pY)
        arrayIn.append(self.pZ)
        arrayIn.append(self.qX)
        arrayIn.append(self.qY)
        arrayIn.append(self.qZ)
        arrayIn.append(self.qW)
    
    def parseDataArray(self, arrayIn):
        self.pX = arrayIn[1]
        self.pY = arrayIn[2]
        self.pZ = arrayIn[3]
        self.qX = arrayIn[4]
        self.qY = arrayIn[5]
        self.qZ = arrayIn[6]
        self.qW = arrayIn[7]

    @staticmethod
    def createSelf(arrayIn):
        return UpdatePosePacket(arrayIn)

RegisterCommandFactoryObj(COMMAND_UPDATE_POSE_TYPE, UpdatePosePacket.createSelf)

class InitialPositionPacket(BaseCommandPacket):
    def __init__(self, dataArrayIn = None):
        self.pX = 0
        self.pY = 0
        super().__init__(dataArrayIn)

    def getPacketTypeID(self):
        return COMMAND_INITIAL_POSITION_TYPE
        
    def encodeToArray(self, arrayIn):
        arrayIn.append(self.pX)
        arrayIn.append(self.pY)
    
    def parseDataArray(self, arrayIn):
        self.pX = arrayIn[1]
        self.pY = arrayIn[2]

    @staticmethod
    def createSelf(arrayIn):
        return InitialPositionPacket(arrayIn)

RegisterCommandFactoryObj(COMMAND_INITIAL_POSITION_TYPE, InitialPositionPacket.createSelf)

# Image Type Definitions
ROBOT_IMG_COMPRESSION_NONE = 0
ROBOT_IMG_COMPRESSION_JPG = 1
ROBOT_IMG_COMPRESSION_JPG_BW = 2

class ImageCommandPacket(BytesCommandPacket):
    def __init__(self, dataArrayIn = None):
        self.compressionType = ROBOT_IMG_COMPRESSION_NONE
        super().__init__(dataArrayIn)

    def getPacketTypeID(self):
        return COMMAND_IMG_TYPE        

    def encodeToArray(self, arrayIn):
        super().encodeToArray(arrayIn)
        arrayIn.append(self.compressionType)
    
    def parseDataArray(self, arrayIn):
        super().parseDataArray(arrayIn)
        self.compressionType = arrayIn[2]

    @staticmethod
    def createSelf(arrayIn):
        return ImageCommandPacket(arrayIn)              

    @staticmethod
    def createImgPacket(dataBytesIn, compressionType=ROBOT_IMG_COMPRESSION_JPG):
        retPkt = ImageCommandPacket()
        retPkt.compressionType = compressionType
        retPkt.bytes = dataBytesIn
        return retPkt

RegisterCommandFactoryObj(COMMAND_IMG_TYPE, ImageCommandPacket.createSelf)       

class ImageListCommandPacket(BaseCommandPacket):
    def __init__(self, dataArrayIn = None):
        self.compressionType = ROBOT_IMG_COMPRESSION_NONE
        self.imageList = []
        super().__init__(dataArrayIn)

    def getPacketTypeID(self):
        return COMMAND_IMG_LIST_TYPE        

    def encodeToArray(self, arrayIn):
        arrayIn.append(self.compressionType)
        arrayIn.append(len(self.imageList))

        for cImage in self.imageList:
            # [(bytes, label) ...]
            cBytes, cLabel = cImage
            arrayIn.append(cBytes)
            arrayIn.append(cLabel)
    
    def parseDataArray(self, arrayIn):
        self.compressionType = arrayIn[1]        
        numImages = arrayIn[2]
        for i in range(0, numImages):
            baseIdx = 3 + (i * 2)
            self.imageList.append((arrayIn[baseIdx], arrayIn[baseIdx + 1]))

    @staticmethod
    def createSelf(arrayIn):
        return ImageListCommandPacket(arrayIn)                  

    @staticmethod
    def createImgListPacket(imgPairListIn, compressionType=ROBOT_IMG_COMPRESSION_JPG):
        '''imgPairListIn is in format: [(imgBytes, label) ...]'''
        retPkt = ImageListCommandPacket()
        retPkt.compressionType = compressionType
        retPkt.imageList = imgPairListIn
        return retPkt

RegisterCommandFactoryObj(COMMAND_IMG_LIST_TYPE, ImageListCommandPacket.createSelf)    

class ImageStreamTogglePacket(BaseCommandPacket):
    def __init__(self, dataArrayIn = None):
        self.toggle = 0
        self.imageNames = []
        super().__init__(dataArrayIn)

    def getPacketTypeID(self):
        return COMMAND_IMG_STREAM_TOGGLE_TYPE
        
    def encodeToArray(self, arrayIn):
        arrayIn.append(self.toggle)
        arrayIn.append(len(self.imageNames))
        for cName in self.imageNames:
            arrayIn.append(cName)
    
    def parseDataArray(self, arrayIn):
        self.toggle = arrayIn[1]
        numNames = arrayIn[2]
        self.imageNames = []
        for i in range(0, numNames):
            self.imageNames.append(arrayIn[3 + i])

    @staticmethod
    def createSelf(arrayIn):
        return ImageStreamTogglePacket(arrayIn)

RegisterCommandFactoryObj(COMMAND_IMG_STREAM_TOGGLE_TYPE, ImageStreamTogglePacket.createSelf)

class ImageListWithMetaCommandPacket(ImageListCommandPacket):
    def __init__(self, dataArrayIn = None):
        self.m_meta = None
        super().__init__(dataArrayIn)

    def getPacketTypeID(self):
        return COMMAND_IMG_LIST_WITH_META_TYPE        

    def encodeToArray(self, arrayIn):
        # TODO: Vincent please fill this in
        pass
    
    def parseDataArray(self, arrayIn):
        # TODO: Vincent please fill this in
        pass

    @staticmethod
    def createSelf(arrayIn):
        return ImageListWithMetaCommandPacket(arrayIn)

RegisterCommandFactoryObj(COMMAND_IMG_LIST_WITH_META_TYPE, ImageListWithMetaCommandPacket.createSelf)

# Server Utils
def getCommandPacketFromArray(arrayIn):
    retObj = None
    objType = arrayIn[0]
    if objType in MsgFactoryMap:
        createFn = MsgFactoryMap[objType]
        retObj = createFn(arrayIn)
     
    return retObj

def processCommandPacketFromBytes(dataBytesIn):
    unpackList = msgpack.unpackb(dataBytesIn, raw=False)
    cmdPacket = getCommandPacketFromArray(unpackList)
    return cmdPacket
    