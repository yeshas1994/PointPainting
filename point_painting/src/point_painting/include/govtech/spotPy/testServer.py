import os
import sys
import asyncio
import websockets
import logging
import msgpack
from random import randrange
import spotMsgs
import threading
import time
import random
import json
import requests
import numpy as np

async def processPackets(websocket, path):
    print("TestServer: A new client connected with path: %s" %path)
    while True:
        dataBytesIn = await websocket.recv()
        cmdPacket = spotMsgs.processCommandPacketFromBytes(dataBytesIn)
        print("%s" %(str(cmdPacket)))

async def runService(websocket, path):
    packetTask = asyncio.create_task(processPackets(websocket, path))
    await packetTask

def startWebSocketsFunc():
    serverAddress = "localhost"
    serverPort = 8768
    print("Starting Test Server with address: %s and port: %d" %(serverAddress, serverPort))
    server = websockets.serve(runService, serverAddress, serverPort)

    asyncio.get_event_loop().run_until_complete(server)
    asyncio.get_event_loop().run_forever()



if __name__ == "__main__":
    startWebSocketsFunc()

        