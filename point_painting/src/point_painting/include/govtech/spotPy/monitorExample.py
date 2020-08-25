from flask import Flask, request
import msgpack
import spotMsgs

app = Flask(__name__)

@app.route('/images', methods=['POST']) 
def foo():
    dataBytes = request.get_data()
    unpackList = msgpack.unpackb(dataBytes, raw=False)
    cmdPacket = spotMsgs.getCommandPacketFromArray(unpackList)
    print("Num images: %s" %(len(cmdPacket.imageList)))
    return "OK"

if __name__ == '__main__':
    app.run()    