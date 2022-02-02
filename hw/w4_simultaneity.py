import sys
from telnetlib import EC
import time
import logging
import argparse
import concurrent.futures

from collections import deque
from readerwriterlock import rwlock

sys.path.append(r'../lib')
from picarx import Picarx
from utils import reset_mcu
from picamera import PiCamera
from picamera.array import PiRGBArray
from w3_sensors import *

reset_mcu()

logging_format = "%(asctime)s: %(message)s"
logging.basicConfig(format=logging_format, level=logging.ERROR , datefmt="%H:%M:%S ")

class Bus():
    def __init__(self):
        self.lock = rwlock.RWLockWriteD()
        self.message = None

    def read(self):
        with self.lock.gen_rlock():
            msg = self.message
        return self.message

    def write(self, message):
        with self.lock.gen_wlock():
            self.message = message