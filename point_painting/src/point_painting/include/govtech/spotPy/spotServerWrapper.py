#!/usr/bin/python3.7

import subprocess
from subprocess import Popen
import sys
import logging
import asyncio

'''
This is a wrapper that will run the SpotServer ( or attempt to ) forever. If the server throws an exception, it will restart it
'''

def printLine(lineIn, logger):
    if lineIn != "":
        print(lineIn, end='')
        logger.info(lineIn)

def runForever(logger):
    while True:
        print("Starting Spot Server...")
        proc = Popen(["/usr/bin/python3.7", "./spotServer.py"], stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines = True)
        while proc.poll() is None:
            line = proc.stdout.readline()
            printLine(line, logger)
            line = proc.stderr.readline()
            printLine(line, logger)

if __name__ == "__main__":
    # Setup logger
    logger = logging.getLogger('spotServerWrapper')
    logger.setLevel(logging.DEBUG)    
    fh = logging.FileHandler('./LogServerWrapper.log')
    fh.setLevel(logging.DEBUG)
    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    fh.setFormatter(formatter)
    logger.addHandler(fh)

    runForever(logger)