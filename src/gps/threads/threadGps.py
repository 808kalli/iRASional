import threading
import base64
import time
import serial
import numpy as np

from src.gps.threads.Gps import Gps

from multiprocessing import Pipe
from src.utils.messages.allMessages import (
    CalcPos,
    Pos,
    Location,
    ImuData
)
from src.templates.threadwithstop import ThreadWithStop

class threadGps(ThreadWithStop):

    # ================================ INIT ===============================================
    def __init__(self, queuesList, logger, debugger):
        super(threadGps, self).__init__()
        self.queuesList = queuesList
        self.logger = logger
        self.debugger = debugger
        self.gps = Gps()



    # =============================== STOP ================================================
    def stop(self):
        super(threadGps, self).stop()

    # =============================== START ===============================================
    def start(self):
        super(threadGps, self).start()

    # =============================== CONFIG ==============================================
    # def Configs(self):
    #     """Callback function for receiving configs on the pipe."""

    # ================================ RUN ================================================
    def run(self):
        """This function will run while the running flag is True"""


        # Configure the Serial Port
        ser = serial.Serial(
            port="/dev/ttyACM1",\
            baudrate=115200,\
            parity=serial.PARITY_NONE,\
            stopbits=serial.STOPBITS_ONE,\
            bytesize=serial.EIGHTBITS,\
            timeout=None)
        print("connected to: " + ser.portstr)

        while self._running:
            self.gps.update(ser)






            