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

async def clientStart():
    uri = "ws://localhost:9022"
    async with websockets.connect(uri) as websocket:
        while True:
            cmdStr = input("Command? ( recordStart, recordStop, recordInfo)")
            respMsg = spotMsgs.MsgStrCommandPacket()
            respMsg.setMsg(cmdStr)
            await websocket.send(respMsg.encode())
            respBytes = await websocket.recv()
            cmdPacket = spotMsgs.processCommandPacketFromBytes(respBytes)
            print("Resp Packet: %s" %(cmdPacket.msg))

if __name__ == "__main__":
    asyncio.get_event_loop().run_until_complete(clientStart())

        