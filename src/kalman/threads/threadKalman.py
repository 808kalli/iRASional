import threading
import base64
import time
import math
import numpy as np
import logging
import networkx as nx

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
        self.angles = []
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
        x_y = 0.9       #initial y position

        #=============DO NOT CHANGES THESE VALUES================
        #IMU acceleration covarriance
        s_angle = 10**(-3.5)

        #GPS observation covarriance
        s_x = 1e-3
        s_y = 1e-3
        #========================================================

        P = np.array([[1e-10, 1e-10],
                      [1e-10, 1e-10]])

        x = np.array([x_x, x_y])

        F = np.array([[1, 0],
                      [0, 1]])

        H = np.array([[1, 0],
                      [0, 1]])

        Q = np.array([[s_angle**2, 0],
                      [0, s_angle**2]])

        R = np.array([[s_x**2, 0],
                    [0, s_y**2]])

        #read graph
        G = nx.read_graphml("./Competition_track_graph.graphml")

        while self._running:
            if self.pipeRecvCurrentSpeed.poll():
                self.vel_y = self.pipeRecvCurrentSpeed.recv()['value']
                v = np.array([0, self.vel_y])

                #Predict step (IMU)
                if self.pipeRecvIMUReading.poll():
                    self.imu = self.pipeRecvIMUReading.recv()['value']
                    magx = float(self.imu['magx'])
                    magz = float(self.imu['magz'])

                    heading = math.atan2(-magx, magz)
                    
                    if heading < 0:
                        heading += 2 * math.pi

                    phi = heading                                   #car angle from north
                    axis_angle_to_north = 4.71239                   #axis angle from north
                    angle = phi - axis_angle_to_north               #car angle to axis

                    if angle < 0:
                        angle += 2 * math.pi

                    Rot = np.array([[np.cos(angle), -np.sin(angle)],
                                    [np.sin(angle), np.cos(angle)]])
                                        
                    x, P = Kalman.predict(x, F, Rot, v, P, Q, dt)         #predict new state

                    coordinates = (x[0], x[1])

                    #find nearest node to current coordinates
                    closest_node = None
                    min_distance = float('inf')

                    for node in G.nodes():
                        node_coordinates = (G.nodes[node]['x'], G.nodes[node]['y'])  # Assuming nodes have 'x' and 'y' attributes
                        distance = np.sqrt((coordinates[0] - node_coordinates[0])**2 + (coordinates[1] - node_coordinates[1])**2)
                        if distance < min_distance:
                            min_distance = distance
                            closest_node = node
                    
                    #send back calculated position
                    self.queuesList[Pos.Queue.value].put(
                        {
                            "Owner": Pos.Owner.value,
                            "msgID": Pos.msgID.value,
                            "msgType": Pos.msgType.value,
                            "msgValue": (coordinates, closest_node)
                        }
                    )

                    self.pipeRecvIMUReading.send("ready")

                #Update step (GPS)
                if self.pipeRecvGPSReading.poll():
                    self.pos = self.pipeRecvGPSReading.recv()['value']/1000     #from mm to m
                    print(self.pos)

                    x, P = Kalman.update(x, self.pos, H, P, R)          #update state

                    self.pipeRecvGPSReading.send("ready")

            self.pipeRecvCurrentSpeed.send("ready")
