#!/usr/bin/env python

# WSS (WS over TLS) server example, with a self-signed certificate
import sys
sys.path.append("../spotPy")
import spotMsgs
import json
import msgpack

import asyncio
import pathlib
import ssl
import websockets

async def hello(websocket, path):
    print("A new client connected with path: %s" %path)

    while(True):
        dataBytesIn = await websocket.recv()
        cmdPacket = spotMsgs.processCommandPacketFromBytes(dataBytesIn)
        print("A packet was received:")
        print(cmdPacket.msg)

useTLS = True

ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
localhost_pem = "./testCerts/serverDefault.pem"
#localhost_pem = "./server.pem"
ssl_context.load_cert_chain(localhost_pem)

if useTLS == False:
    ssl_context = None

start_server = websockets.serve(
    hello, "localhost", 8765, ssl=ssl_context
)

if useTLS:
    print("Starting Secure Websockets Server.")
else:
    print ("Starting Non-Secure Websockets Server")

asyncio.get_event_loop().run_until_complete(start_server)
asyncio.get_event_loop().run_forever()