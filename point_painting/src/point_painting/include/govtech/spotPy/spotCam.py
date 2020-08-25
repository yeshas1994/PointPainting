from media_log import MediaLogClient
from bosdyn.api.spot_cam import logging_pb2, camera_pb2
from bosdyn.client.spot_cam.lighting import LightingClient
import os
import shutil
import tempfile
import threading
import time
import cv2
import numpy as np
from PIL import Image
from bosdyn.client import spot_cam

class SpotCam(object):
    def __init__(self, storeFolder, robotIn, finishFn=None):
        self._storeFolder = storeFolder
        try:
            os.mkdir(storeFolder)
        except:
            pass

        print("Storing SpotCam images in %s" %self._storeFolder)
        self._recordImages = 0
        self._streamImg = None
        self.robot = robotIn
        self._compressVal = 40
        self._storeCnter = 0
        self._finishFn = finishFn

        self._lps = []
        self._lpsLock = threading.Lock()

        self._runStream = True
        self._streamThread = threading.Thread(target = self.snapFunc)
        self._streamThread.start()
        self._streamLock = threading.Lock()
        print("SpotCam Started.")
    
    def setCompressVal(self, valIn):
        self._compressVal = valIn

    def isRecording(self):
        return self._recordImages > 0

    def getStoreFolder(self):
        return self._storeFolder

    def setRecordImages(self, flagIn):
        if flagIn:
            self._recordImages = 1
        else:
            self._recordImages = 0
    
    def stopStream(self):
        self._runStream = False

    def snapFunc(self):
        print("Taking SpotCam Snapshot...")
        self.retrieveAndStore(self.robot)
        print("SpotCam Stopped.")
        if self._finishFn:
            self._finishFn()
    
    @staticmethod
    def setLighting(robotIn, flagIn):
        lightVal = 0.0
        if flagIn:
            lightVal = 1.0

        robotIn.ensure_client(LightingClient.default_service_name).set_led_brightness(
            [lightVal, lightVal, lightVal, lightVal])

    def getUniqueSaveFilename(self):
        storeCnter = 0
        fileExists = True
        outFilename = ""
        # Find a unique filename to save to
        while fileExists == True:
            outFilename = self._storeFolder + "/" + "img_" + str(storeCnter)
            fileExists = os.path.exists(outFilename + ".jpg")
            storeCnter += 1
        return outFilename

    def saveLPsFunc(self):
        storeCnter = 0
        sLP = None            
        with self._lpsLock:
            if len(self._lps) > 0:
                sLP = self._lps.pop(0)
        if sLP != None:
            fileExists = True
            outFilename = self.getUniqueSaveFilename()
            self.save_logpoint_as_image(sLP, self.robot, True, False, dst_filename=outFilename, recordImages=self._recordImages)
            print("Saved Log Image: %s" %outFilename)

    
    def getStreamImg(self):
        with self._streamLock:
            return self._streamImg

    def retrieveAndStore(self, robot):
        """Retrieves an image from the SpotCam and saves it out"""
        client = robot.ensure_client(MediaLogClient.default_service_name)
        args = (camera_pb2.Camera(name='pano'), logging_pb2.Logpoint.STILLIMAGE)
        lp = client.store(*args)

        while lp.status != logging_pb2.Logpoint.COMPLETE:
            lp = client.get_status(lp)

        outFilename = self.getUniqueSaveFilename()
        self.save_logpoint_as_image(lp, self.robot, True, False, dst_filename=outFilename, recordImages=True)
        print("Saved Log Image: %s" %outFilename)

    def save_logpoint_as_image(self, lp, robot, stitching, save_as_rgb24, dst_filename, recordImages):
        """save-as-rgb24 and stitching."""
        if stitching:
            lp, img = robot.ensure_client(MediaLogClient.default_service_name).retrieve(lp)
        else:
            lp, img = robot.ensure_client(MediaLogClient.default_service_name).retrieve_raw_data(lp)

        imgArray = np.frombuffer(img, dtype=np.uint8)
        imgOrig = cv2.imdecode(imgArray, cv2.IMREAD_COLOR)
        imgCV = cv2.resize(imgOrig, (960, 480))
        imgCV = cv2.cvtColor(imgCV, cv2.COLOR_BGR2RGB)

        with self._streamLock:
            self._streamImg = cv2.imencode('.jpg', imgCV, [cv2.IMWRITE_JPEG_QUALITY, self._compressVal])

        if recordImages == False:
            return None

        print("Recording Image to %s" %dst_filename)

        with tempfile.NamedTemporaryFile(delete=False) as img_file:
            img_file.write(img)
            src_filename = img_file.name

        if dst_filename is None:
            dst_filename = os.path.basename(src_filename)

        if lp.image_params.height == 4800:
            shutil.move(src_filename, '{}.jpg'.format(dst_filename))
        else:
            target_filename = '{}-{}x{}.rgb24'.format(dst_filename, lp.image_params.width, lp.image_params.height)
            shutil.move(src_filename, target_filename)

            if not save_as_rgb24:
                with open(target_filename, mode='rb') as fd:
                    data = fd.read()

                mode = 'RGB'
                image = Image.frombuffer(mode, (lp.image_params.width, lp.image_params.height), data, 'raw', mode, 0, 1)
                image.save('{}.jpg'.format(dst_filename))

                os.remove(target_filename)