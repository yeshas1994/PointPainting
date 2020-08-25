
# WSS (WS over TLS) client example, with a self-signed certificate

import sys
sys.path.append("../spotPy")
import spotMsgs
import json
import msgpack
import asyncio
import pathlib
import ssl
import websockets
import time

ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
#localhost_pem = "./testCerts/ca-chain.cert.pem"
localhost_pem = "./testCerts/serverDefault.pem"
ssl_context.load_verify_locations(localhost_pem)

async def hello():
    uri = "wss://localhost:8765"
    async with websockets.connect(
        uri, ssl=ssl_context
    ) as websocket:

        while True:
            respMsg = spotMsgs.MsgStrCommandPacket()
            respMsg.setMsg("Hello this is a a test!")
            await websocket.send(respMsg.encode())
            time.sleep(2)

asyncio.get_event_loop().run_until_complete(hello())