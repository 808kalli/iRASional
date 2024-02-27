import threading
import base64
import time
import numpy as np
import logging

import src.kalman.threads.Kalman as Kalman 

from multiprocessing import Pipe
from src.utils.messages.allMessages import (
    CalcPos,
    Pos,
    Location,
    ImuData
)
from src.templates.threadwithstop import ThreadWithStop

class threadKalman(ThreadWithStop):

    # ================================ INIT ===============================================
    def __init__(self, queuesList, logger, debugger):
        super(threadKalman, self).__init__()
        self.queuesList = queuesList
        self.logger = logger
        self.debugger = debugger
        pipeRecvCalcPos, pipeSendCalcPos = Pipe()
        pipeRecvIMUReading, pipeSendIMUReading = Pipe()
        pipeRecvGPSReading, pipeSendGPSReading = Pipe()
        self.pipeRecvGPSReading = pipeRecvGPSReading
        self.pipeSendGPSReading = pipeSendGPSReading
        self.pipeRecvIMUReading = pipeRecvIMUReading
        self.pipeSendIMUReading = pipeSendIMUReading
        self.pipeRecvCalcPos = pipeRecvCalcPos
        self.pipeSendCalcPos = pipeSendCalcPos
        self.subscribe()
        self.pipeRecvCalcPos.send("ready")   #send ready flag through pipe
        self.pipeRecvIMUReading.send("ready")
        self.pipeRecvGPSReading.send("ready")

    def subscribe(self):
        """Subscribe function. In this function we make all the required subscribe to process gateway"""
        self.queuesList["Config"].put(
            {
                "Subscribe/Unsubscribe": "subscribe",
                "Owner": CalcPos.Owner.value,
                "msgID": CalcPos.msgID.value,
                "To": {"receiver": "threadPathPlanning", "pipe": self.pipeSendCalcPos},
            }
        )

        self.queuesList["Config"].put(
            {
                "Subscribe/Unsubscribe": "subscribe",
                "Owner": ImuData.Owner.value,
                "msgID": ImuData.msgID.value,
                "To": {"receiver": "threadPathPlanning", "pipe": self.pipeSendIMUReading},
            }
        )

        self.queuesList["Config"].put(
            {
                "Subscribe/Unsubscribe": "subscribe",
                "Owner": Location.Owner.value,
                "msgID": Location.msgID.value,
                "To": {"receiver": "threadPathPlanning", "pipe": self.pipeSendGPSReading},
            }
        )

    '''
    Maybe add this function later, if we want to send back an
    acknolegdment message, saying that we recieved the 'record' flag
    '''
    # def Queue_Sending(self):
    #     """Callback function for recording flag."""
    #     self.queuesList[Recording.Queue.value].put(
    #         {
    #             "Owner": Recording.Owner.value,
    #             "msgID": Recording.msgID.value,
    #             "msgType": Recording.msgType.value,
    #             "msgValue": self.recording,
    #         }
    #     )
    #     threading.Timer(1, self.Queue_Sending).start()

    # =============================== STOP ================================================
    def stop(self):
        super(threadKalman, self).stop()

    # =============================== START ===============================================
    def start(self):
        super(threadKalman, self).start()

    # =============================== CONFIG ==============================================
    # def Configs(self):
    #     """Callback function for receiving configs on the pipe."""

    # ================================ RUN ================================================
    def run(self):
        """This function will run while the running flag is True"""
        
        #initial values
        dt = 0.1        #time interval
        x_x = 0
                 #initial x position
        x_y = 0         #initial y position
        u0_x = 0        #initial x velocity
        u0_y = 0        #initial y velocity
        a_x = 0         #initial x acceleration
        a_y = 0         #initial y acceleration

        #IMU acceleration covarriance
        s_a_x = 1e-2
        s_a_y = 1e-2

        #GPS observation covarriance
        s_x = 1e-1
        s_y = 1e-1

        P = np.array([[1e-10, 1e-10, 1e-10, 1e-10],
                    [1e-10, 1e-10, 1e-10, 1e-10],
                    [1e-10, 1e-10, 1e-10, 1e-10],
                    [1e-10, 1e-10, 1e-10, 1e-10]])

        accel = np.array([a_x, a_y])

        x = np.array([x_x, x_y, u0_x, u0_y])

        F = np.array([[1, 0, dt, 0],
                    [0, 1, 0, dt],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])

        G = np.array([[0.5*dt**2, 0],
                    [0, 0.5*dt**2],
                    [dt, 0],
                    [0, dt]])

        H = np.array([[1, 0, 0, 0],
                    [0, 1, 0, 0]])

        Q = np.array([[0.25*dt**4*s_a_x**2, 0, 0.5*dt**3*s_a_x**2, 0],
                    [0, 0.25*dt**4*s_a_y**2, 0, 0.5*dt**3*s_a_y**2],
                    [0.5*dt**3*s_a_x**2, 0, dt**2*s_a_x**2, 0],
                    [0, 0.5*dt**3*s_a_y**2, 0, dt**2*s_a_y**2]])

        R = np.array([[s_x**2, 0],
                    [0, s_y**2]])

        while self._running:
            if self.pipeRecvCalcPos.poll():
                self.initiate = self.pipeRecvCalcPos.recv()
                if self.initiate['value'] == True:
                    while self._running:
                        if self.pipeRecvIMUReading.poll():
                            self.imu = self.pipeRecvIMUReading.recv()['value']
                            magx = float(self.imu['magx'])

                            print(magx)

                            self.pipeRecvIMUReading.send("ready")

                        if self.pipeRecvGPSReading.poll():
                            self.pos = self.pipeRecvGPSReading.recv()['value']/1000

                            x, P = Kalman.update(x, self.pos, H, P, R)          #update state

                            self.pipeRecvGPSReading.send("ready")

                            coordinates = (x[0], x[1])

                            self.queuesList[Pos.Queue.value].put(      #send back calculated position
                                {
                                    "Owner": Pos.Owner.value,
                                    "msgID": Pos.msgID.value,
                                    "msgType": Pos.msgType.value,
                                    "msgValue": self.pos
                                }
                            )