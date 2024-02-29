import threading
import base64
import time
import math
import numpy as np
import logging

import src.kalman.threads.Kalman as Kalman 

from multiprocessing import Pipe
from src.utils.messages.allMessages import (
    CurrentSpeed,
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
        pipeRecvCurrentSpeed, pipeSendCurrentSpeed = Pipe()
        pipeRecvIMUReading, pipeSendIMUReading = Pipe()
        pipeRecvGPSReading, pipeSendGPSReading = Pipe()
        self.pipeRecvGPSReading = pipeRecvGPSReading
        self.pipeSendGPSReading = pipeSendGPSReading
        self.pipeRecvIMUReading = pipeRecvIMUReading
        self.pipeSendIMUReading = pipeSendIMUReading
        self.pipeRecvCurrentSpeed = pipeRecvCurrentSpeed
        self.pipeSendCurrentSpeed = pipeSendCurrentSpeed
        self.subscribe()
        self.pipeRecvCurrentSpeed.send("ready")   #send ready flag through pipe
        self.pipeRecvIMUReading.send("ready")
        self.pipeRecvGPSReading.send("ready")


    def subscribe(self):
        """Subscribe function. In this function we make all the required subscribe to process gateway"""
        self.queuesList["Config"].put(
            {
                "Subscribe/Unsubscribe": "subscribe",
                "Owner": CurrentSpeed.Owner.value,
                "msgID": CurrentSpeed.msgID.value,
                "To": {"receiver": "threadPathPlanning", "pipe": self.pipeSendCurrentSpeed},
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
        dt = 0.5        #time interval
        x_x = 4.12      #initial x position
        x_y = 1         #initial y position

        #=============DO NOT CHANGES THESE VALUES================
        #IMU acceleration covarriance
        s_angle = 1e-5

        #GPS observation covarriance
        s_x = 1e-3  
        s_y = 1e-4
        #========================================================

        P = np.array([[1e-10, 1e-10],
                      [1e-10, 1e-10]])

        x = np.array([x_x, x_y])

        F = np.array([[1, 0],
                      [0, 1]])

        H = np.array([[1, 0],
                      [0, 1]])

        Q = np.array([[s_angle**2, s_angle**2],
                      [s_angle**2, s_angle**2]])

        R = np.array([[s_x**2, 0],
                    [0, s_y**2]])

        while self._running:
            if self.pipeRecvCurrentSpeed.poll():
                self.vel_y = self.pipeRecvCurrentSpeed.recv()['value']
                v = np.array([0, self.vel_y])

                #Predict step (IMU)
                if self.pipeRecvIMUReading.poll():
                    self.imu = self.pipeRecvIMUReading.recv()['value']
                    magx = float(self.imu['magx'])
                    magy = float(self.imu['magy'])

                    heading = math.atan2(magy, magx)
                    
                    if heading < 0:
                        heading += 2 * math.pi

                    phi = heading                                   #car angle from north
                    axis_angle_to_north = 1.43302004                #axis angle from north
                    angle = axis_angle_to_north - phi               #car angle to axis

                    if angle < 0:
                        angle += 2 * math.pi

                    Rot = np.array([[np.cos(angle), -np.sin(angle)],
                                    [np.sin(angle), np.cos(angle)]])
                                        
                    x, P = Kalman.predict(x, F, Rot, v, P, Q, dt)         #predict new state

                    coordinates = (x[0], x[1])

                    self.queuesList[Pos.Queue.value].put(      #send back calculated position
                        {
                            "Owner": Pos.Owner.value,
                            "msgID": Pos.msgID.value,
                            "msgType": Pos.msgType.value,
                            "msgValue": coordinates
                        }
                    )

                    self.pipeRecvIMUReading.send("ready")

                #Update step (GPS)
                if self.pipeRecvGPSReading.poll():
                    self.pos = self.pipeRecvGPSReading.recv()['value']/1000     #from mm to m

                    x, P = Kalman.update(x, self.pos, H, P, R)          #update state

                    self.pipeRecvGPSReading.send("ready")

            self.pipeRecvCurrentSpeed.send("ready")