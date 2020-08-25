import msgpack
import os
import sys
import cv2
import numpy as np

def writeBinFile(byesIn, filenameIn):
    with open(filenameIn,'wb') as f:
        f.write(byesIn)

def readPackfromFilename(filenameIn, imgFolder, cnt):
    # Read msgpack file
    with open(filenameIn, "rb") as data_file:
        byteData = data_file.read()

    packData = msgpack.unpackb(byteData)
    # Data is packed in the following way: (Position, Orientation, Velocities, Camera Image, Time)
    '''
        px
        py
        pz

        qx
        qy
        qz
        qw

        moveVX
        moveVY
        moveVROT

        cameraSpotCam
        
        time 
    '''
    px = packData[0]
    py = packData[1]
    pz = packData[2]        

    qx = packData[3]
    qy = packData[4]
    qz = packData[5]
    qw = packData[6]                        

    moveVX = packData[7]        
    moveVY = packData[8]        
    moveVROT = packData[9]        

    camImg = packData[10]        
    
    timeStr = str(packData[11])

    print("Record State Object %d data:" %cnt)
    print("[%d] Pos: (%f, %f, %f), Quat: (%f, %f, %f, %f), Vels: (%f, %f, %f)" %(cnt, px, py, pz, qx, qy, qz, qw, moveVX, moveVY, moveVROT))
    print("Time: %s" %timeStr)

    # Write images
    jpgFilename = imgFolder + "/cam_" + str(cnt) + ".jpg"
    camBytes = np.frombuffer(camImg, dtype=np.int8)
    cImg = cv2.imdecode(camBytes, cv2.IMREAD_COLOR)
    imgCV = cv2.cvtColor(cImg, cv2.COLOR_BGR2RGB)
    cv2.imwrite(jpgFilename, imgCV)
    print("Writing jpg to: %s" %jpgFilename)

if __name__ == "__main__":
    for i in range(0, 400):
        readPackfromFilename('../../spotControlStateRuns/record_0.stateRun/data_' + str(i) + '.pack', '../../spotControlStateRuns/unpack', i)