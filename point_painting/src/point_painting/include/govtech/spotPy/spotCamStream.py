import asyncio
import base64
import json
import requests
import threading
import cv2
import numpy as np
import time
import os
import queue
from bosdyn.client.spot_cam.streamquality import StreamQualityClient
from bosdyn.api.spot_cam import streamquality_pb2

from aiortc import (
    RTCConfiguration,
    RTCPeerConnection,
    RTCSessionDescription,
    MediaStreamTrack,
)

class RTCOptions(object):
    def __init__(self, hostnameIn):
        self.track = "video"
        self.filename = "h264.sdp"
        self.sdp_port = 31102
        self.cam_ssl_cert = False
        self.dst_prefix = "h264.sdp"
        self.hostname = hostnameIn

class WebRTCStreamer(object):
    """Streaming the RTC stream data"""
    def __init__(self, folderIn, hostname="192.168.80.3"):
        self.options = RTCOptions(hostname)
        self.imgStreamLock = threading.Lock()
        self.imgStreamArray = None
        self.imgStreamArrayValid = False
        self.imgStreamRun = True
        self.imgStreamFolder = folderIn
        self.imgStreamRecord = False
        self.imgStreamFilename = None
        
        self.fourcc = None
        self.videoOut = None

        print("Storing SpotCAM Stream in folder: %s with hostname: %s" %(self.imgStreamFolder, hostname))

    def getUniqueStreamFilename(self):
        storeCnter = 0
        fileExists = True
        outFilename = ""
        # Find a unique filename to save to
        while fileExists == True:
            outFilename = self.imgStreamFolder + "/" + "stream_" + str(storeCnter) + ".mp4"
            fileExists = os.path.exists(outFilename)
            storeCnter += 1
        return outFilename

    def setRecord(self, flagIn):
        self.imgStreamRecord = flagIn
        if self.imgStreamRecord:
            # Setup video encoding
            self.imgStreamFilename = self.getUniqueStreamFilename()
            self.fourcc = cv2.VideoWriter_fourcc(*'MP4V')
            self.videoOut = cv2.VideoWriter(self.imgStreamFilename, self.fourcc, 15.0, (1280,720))
            print("Starting Video Recording of SpotCamStream to: %s" %self.imgStreamFilename)
        else:
            # Stop video encoding
            if self.videoOut and self.fourcc:
                self.videoOut.release()
                self.fourcc = None
                self.videoOut = None
                print("Finished Video Recording of SpotCamStream to: %s" %self.imgStreamFilename)
                self.imgStreamFilename = None

    def isRecording(self):
        return self.imgStreamRecord

    def stopStream(self):
        self.imgStreamRun = False
       
    def getImgStreamBytes(self, compressionLevel=40):
        with self.imgStreamLock:
            cImgData = self.imgStreamArray
            if self.imgStreamArrayValid == False:
                cImgData = np.zeros((960, 480, 3), np.uint8)
                return cv2.imencode('.jpg', cImgData, [cv2.IMWRITE_JPEG_QUALITY, compressionLevel])

            imgCV = cv2.resize(cImgData, (960, 480))
            imgCV = cv2.cvtColor(imgCV, cv2.COLOR_BGR2RGB)
            return cv2.imencode('.jpg', imgCV, [cv2.IMWRITE_JPEG_QUALITY, compressionLevel])

    def run(self, robot, target_bitrate=2500000, refresh_interval=30, idr_interval=30, awb="AUTO"):
        robot.ensure_client(StreamQualityClient.default_service_name).set_stream_params(target_bitrate, refresh_interval, idr_interval, awb)
        streamSettings = robot.ensure_client(StreamQualityClient.default_service_name).get_stream_params()
        print("Camera Stream settings: \n %s" %str(streamSettings))

        if not self.options.cam_ssl_cert:
          self.options.cam_ssl_cert = False

        shutdown_flag = threading.Event()
        webrtc_thread = threading.Thread(
            target=start_webrtc,
            args=[shutdown_flag, self.options, self],
            daemon=True
        )
        webrtc_thread.start()
        print("Started SpotCAM Camera Streaming.") 

# WebRTC must be in its own thread with its own event loop.
def start_webrtc(shutdown_flag, options, src_obj):
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    config = RTCConfiguration(iceServers=[])
    client = SpotCAMWebRTCClient(options, config)

    asyncio.gather(client.start(),
                   process_frame(client, options, shutdown_flag, src_obj),
                   monitor_shutdown(shutdown_flag, client))
    loop.run_forever()

# Frame processing occurs; otherwise it waits.
async def process_frame(client, options, shutdown_flag, src_obj):
    count = 0
    while asyncio.get_event_loop().is_running():
        try:
            frame = await client.video_frame_queue.get()
            with src_obj.imgStreamLock:
                src_obj.imgStreamArray = frame.to_ndarray(format="bgr24")
                src_obj.imgStreamArrayValid = True

                if src_obj.imgStreamRecord == True:
                    src_obj.videoOut.write(src_obj.imgStreamArray)

                #print("Received stream %d" %count)

            count += 1
        except:
            pass
        
        if src_obj.imgStreamRun == False:
            break

    print("Stopped SpotCAM Camera Streaming.") 
    shutdown_flag.set()

# Flag must be monitored in a different coroutine and sleep to allow frame
# processing to occur.
async def monitor_shutdown(shutdown_flag, client):
    while not shutdown_flag.is_set():
        await asyncio.sleep(1.0)

    await client.pc.close()
    asyncio.get_event_loop().stop()

def get_sdp_offer_from_spot_cam(options):
    server_url = f'https://{options.hostname}:{options.sdp_port}/{options.filename}'
    r = requests.get(server_url, verify=options.cam_ssl_cert)
    result = r.json()
    return result['id'], base64.b64decode(result['sdp']).decode()

def send_sdp_answer_to_spot_cam(options, offer_id, sdp_answer):
    server_url = f'https://{options.hostname}:{options.sdp_port}/{options.filename}'

    payload = {'id': offer_id, 'sdp': base64.b64encode(sdp_answer).decode('utf8')}
    #r = requests.post(server_url, verify=options.cam_ssl_cert, json=payload)
    jsonStr = json.dumps(payload)
    r = requests.post(server_url, verify=options.cam_ssl_cert, data=jsonStr)
    if r.status_code != 200:
        raise ValueError(r)

class SpotCAMMediaStreamTrack(MediaStreamTrack):
    def __init__(self, track, queue):
        super().__init__()

        self.track = track
        self.queue = queue

    async def recv(self):
        frame = await self.track.recv()

        await self.queue.put(frame)

        return frame

class SpotCAMWebRTCClient:
    def __init__(self, options, rtc_config):
        self.pc = RTCPeerConnection(configuration=rtc_config)

        self.video_frame_queue = asyncio.Queue()
        self.audio_frame_queue = asyncio.Queue()

        self.options = options

    async def start(self):
        #offer_id = None
        offer_id, sdp_offer = get_sdp_offer_from_spot_cam(self.options)

        @self.pc.on('icegatheringstatechange')
        def _on_ice_gathering_state_change():
            print(f'ICE gathering state changed to {self.pc.iceGatheringState}')

        @self.pc.on('signalingstatechange')
        def _on_signaling_state_change():
            print(f'Signaling state changed to: {self.pc.signalingState}')

        @self.pc.on('icecandidate')
        def _on_ice_candidate(event):
            print(f'Received candidate: {event.candidate}')

        @self.pc.on('iceconnectionstatechange')
        async def _on_ice_connection_state_change():
            print(f'ICE connection state changed to: {self.pc.iceConnectionState}')

            if self.pc.iceConnectionState == 'checking':
                send_sdp_answer_to_spot_cam(self.options, offer_id, self.pc.localDescription.sdp.encode())

        @self.pc.on('track')
        def _on_track(track):
            print(f'Received track: {track.kind}')

            if track.kind == 'video':
                video_track = SpotCAMMediaStreamTrack(track, self.video_frame_queue)
                video_track.kind = 'video'
                self.pc.addTrack(video_track)
            elif track.kind == 'audio':
                audio_track = SpotCAMMediaStreamTrack(track, self.audio_frame_queue)
                audio_track.kind = 'audio'
                self.pc.addTrack(audio_track)

        desc = RTCSessionDescription(sdp_offer, 'offer')
        await self.pc.setRemoteDescription(desc)

        sdp_answer = await self.pc.createAnswer()
        await self.pc.setLocalDescription(sdp_answer)

if __name__ == "__main__":
    camStream = WebRTCStreamer()
    camStream.run(None)
    while(True):
        time.sleep(1)

