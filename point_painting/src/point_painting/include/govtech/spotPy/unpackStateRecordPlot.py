import matplotlib.path as mpath
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.lines as mlines
import math
import msgpack
import json
import numpy as np
import quaternion
import cv2

import os
import pandas as pd
from skimage import io, transform
import numpy as np
import matplotlib.pyplot as plt

fig, ax = plt.subplots()
srcFolder = '../../spotControlStateRuns/'
packData = None
minX = float("inf")
maxX = -float("inf")
minY = float("inf")
maxY = -float("inf")

minVelX = float("inf")
maxVelX = -float("inf")
minVelY = float("inf")
maxVelY = -float("inf")
minAngVel = float("inf")
maxAngVel = -float("inf")

nData = 100

def rotatePt(origin, point, angle):
    """
    Rotate a point counterclockwise by a given angle around a given origin.

    The angle should be given in radians.
    """
    ox = origin[0]
    oy = origin[1]

    px = point[0]
    py = point[1]

    qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
    qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
    return np.array([qx, qy, 0.0])

def getPackFilename(cnt):
    return '../../spotControlStateRuns/record_0.stateRun/data_' + str(cnt) + '.pack'

def getPackData(cnt):
    with open(getPackFilename(cnt), mode='rb') as data_file:
        return msgpack.unpackb(data_file.read(), raw=False)

packDataList = []
for i in range(0, nData):
    srcFilename = getPackFilename(i)
    with open(srcFilename, mode='rb') as data_file:
        packData = getPackData(i)
        packDataList.append(packData)
        print("Read data from: %s" %srcFilename)
        print("Num points: %d" %(len(packData) / 12))

        print("Computing Path Bounding Box...")
        for i in range(0, len(packData), 12):
            minX = min(minX, packData[0])
            maxX = max(maxX, packData[0])
            minY = min(minY, packData[1])
            maxY = max(maxY, packData[1])

            minVelX = min(minVelX, packData[7])
            maxVelX = max(maxVelX, packData[7])
            minVelY = min(minVelY, packData[8])
            maxVelY = max(maxVelY, packData[8])
            minAngVel = min(minAngVel, packData[9])
            maxAngVel = max(maxAngVel, packData[9])
        
lengthX = maxX - minX
lengthY = maxY - minY
print("Bounding Box: (%f, %f) - (%f, %f), LengthX: %f, LengthX: %f" %(minX, minY, maxX, maxY, lengthX, lengthY))
print("Min/Max VelX, VelY, AngVel: (%f, %f, %f) - (%f, %f, %f)" %(minVelX, minVelY, minAngVel, maxVelX, maxVelY, maxAngVel))

Path = None
pathPts = None
showImgs = True
imgList = []

def createPath():
    global Path
    global pathPts
    global showImgs

    Path = mpath.Path
    path_data = []
    for i in range(0, len(packDataList)):
        packData = packDataList[i]
        curX = packData[0]
        curY = packData[1]

        if i == 0:
            path_data.append((Path.MOVETO, (curX, curY)))
        else:
            path_data.append((Path.LINETO, (curX, curY)))

        if showImgs:
            camImg = packData[10]
            camBytes = np.frombuffer(camImg, dtype=np.int8)
            cImg = cv2.imdecode(camBytes, cv2.IMREAD_COLOR)
            imgCV = cv2.cvtColor(cImg, cv2.COLOR_BGR2RGB)
            imgList.append((curX, curY, imgCV))

        '''
        self.minVelX = min(self.minVelX, self.packData[i + 7])
        self.maxVelX = max(self.maxVelX, self.packData[i + 7])
        self.minVelY = min(self.minVelY, self.packData[i + 8])
        self.maxVelY = max(self.maxVelY, self.packData[i + 8])
        self.minAngVel = min(self.minAngVel, self.packData[i + 9])
        self.maxAngVel = max(self.maxAngVel, self.packData[i + 9])    
        '''

    '''
    path_data = [
        (Path.MOVETO, (1.58, -2.57)),
        (Path.CURVE4, (0.35, -1.1)),
        (Path.CURVE4, (-1.75, 2.0)),
        (Path.CURVE4, (0.375, 2.0)),
        (Path.LINETO, (0.85, 1.15)),
        (Path.CURVE4, (2.2, 3.2)),
        (Path.CURVE4, (3, 0.05)),
        (Path.CURVE4, (2.0, -0.5)),
        (Path.CLOSEPOLY, (1.58, -2.57)),
        ]
    '''

    codes, verts = zip(*path_data)
    path = mpath.Path(verts, codes)
    x, y = zip(*path.vertices)
    pathPts = (x, y)

createPath()
def animate(i):
    global imgList
    ax.clear()
    line, = ax.plot(pathPts[0], pathPts[1], 'go--')

    # From data
    #rIdx = jsonData["idx"] * 12
    rIdx = 0

    # Input: px, py, pz, qx, qy, qz, qw
    baseOdom = np.array([packData[rIdx], packData[rIdx + 1], packData[rIdx + 2], packData[rIdx + 3], packData[rIdx + 4], packData[rIdx + 5], packData[rIdx + 6]])
    # Output: moveVx, moveVrot
    baseControl = np.array([packData[rIdx + 7], packData[rIdx + 9]])
    
    # Transformed Input: px, py, dirX, dirY
    baseQuat = np.quaternion(packData[rIdx + 6], packData[rIdx + 3], packData[rIdx + 4], packData[rIdx + 5])
    baseMat = quaternion.as_rotation_matrix(baseQuat)
    plotDir = baseMat.dot(np.array([1.0, 0.0, 0.0]))
    angOffset = 0
    plotDir = rotatePt((0,0), plotDir, angOffset)

    plotPt = [baseOdom[0], baseOdom[1]]
    cv2.imshow("Video Record", imgList[i % nData][2])

    '''
    plt.scatter(plotPt[0], plotPt[1], s=200)

    # Inference
    #angleOutput = inferAngleModel(modelAngle, plotPt[0], plotPt[1], plotDir[0], plotDir[1])
    #print("Angle Output: %s " %angleOutput)

    
    for m in range(0, 12):
        factor = m / 10.0 * 3.0 + 0.1
        plt.plot(
            plotPt[0], 
            plotPt[1], 
            [plotPt[0] + plotDir[0] * factor],
            [plotPt[1] + plotDir[1] * factor], 
            marker = 'o', color='orange')
    '''

    ax.grid()
    ax.axis('equal')

ani = animation.FuncAnimation(fig, animate, interval=100) 

plt.show()