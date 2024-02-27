# Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
import threading
import time
import cv2
import numpy as np
import base64
import logging
from collections import deque

from multiprocessing import Pipe
from src.utils.messages.allMessages import (
    ImuData, 
    serialCamera,
    EngineRun,
    TrafficSign,
    BatteryLvl,
    Pedestrian,
    Path,
    Calculate,
    Estimate,
    InterDistance,
    MoveConfig,
    CalcPos,
    Pos,
    Location
)
from src.templates.threadwithstop import ThreadWithStop

from src.move.threads.movements.basic import setSpeed, steer, brake, start_recording, stop_recording
import src.move.threads.movements.lane_following as lf
from src.move.threads.movements.sign_reaction import sign_reaction
from src.move.threads.movements.pedestrian_reaction import pedestrian_reaction
from src.move.threads.movements.parking_test import park_the_car
from src.move.threads.movements.intersection import gostraight, right_turn, left_turn, intersection_navigation
from src.move.threads.movements.roundabout_reaction import roundabout
class threadMove(ThreadWithStop):
    """Thread which will handle the decision making.\n
    Args:
        pipeRecv: maybe
        pipeSend: maybe
        queuesList (dictionar of multiprocessing.queues.Queue): Dictionar of queues where the ID is the type of messages.
        logger (logging object): Made for debugging.
        debugger (bool): A flag for debugging.
    """

    # ================================ INIT ===============================================
    def __init__(self, queuesList, logger, debugger):
        super(threadMove, self).__init__()
        self.queuesList = queuesList
        self.logger = logger
        self.debugger = debugger
        
        # setting the parameters needed
        self.engine = False
        
        # Setting the parameters for lane following
        self.K = 0.15 #0.12 or 0.15 maybe works
        self.speed = 15 #15 is default
        
        # flags
        self.autonomous = True
        self.recording = True

        
        self.pipes = list()
        pipeRecvstart, pipeSendstart = Pipe()
        self.pipeRecvstart = pipeRecvstart
        self.pipeSendstart = pipeSendstart
        self.pipeRecvstart.send("ready")
        # self.pipeRecvstart will not be appended to self.pipes, we want it to work separately
        pipeRecvconfig, pipeSendconfig = Pipe()
        self.pipeRecvconfig = pipeRecvconfig
        self.pipeSendconfig = pipeSendconfig
        self.pipeRecvconfig.send("ready")
        # same for pipeRecvconfig
        
        pipeRecvPathPlanning, pipeSendPathPlanning = Pipe(duplex = True)
        pipeRecvInterDet, pipeSendInterDet = Pipe(duplex = True)
        pipeRecvcamera_lf, pipeSendcamera_lf = Pipe()
        pipeIMUrecv, pipeIMUsend = Pipe()
        pipeRecvPos, pipeSendPos = Pipe()
        self.pipeIMUrecv = pipeIMUrecv
        self.pipes.append(self.pipeIMUrecv)
        self.pipeIMUsend = pipeIMUsend       
        self.pipeRecvcamera_lf = pipeRecvcamera_lf
        self.pipes.append(self.pipeRecvcamera_lf)
        self.pipeSendcamera_lf = pipeSendcamera_lf
        piperecvTrSigns, pipesendTrSigns = Pipe()
        self.piperecvTrSigns = piperecvTrSigns
        self.pipes.append(self.piperecvTrSigns)
        self.pipesendTrSigns = pipesendTrSigns
        piperecvPed,pipesendPed = Pipe()
        self.piperecvPed = piperecvPed
        self.pipesendPed = pipesendPed
        self.pipes.append(self.piperecvPed)
        self.pipeRecvPathPlanning = pipeRecvPathPlanning
        self.pipeSendPathPlanning = pipeSendPathPlanning
        self.pipes.append(self.pipeRecvPathPlanning)
        self.pipeRecvInterDet = pipeRecvInterDet
        self.pipeSendInterDet = pipeSendInterDet
        self.pipes.append(self.pipeRecvInterDet)
        self.pipeRecvPos = pipeRecvPos
        self.pipeSendPos = pipeSendPos
        self.pipes.append(self.pipeRecvPos)
        self.subscribe()
        for pipe in self.pipes:
            pipe.send("ready")

    def subscribe(self):
        """Subscribe function. In this function we make all the required subscribe to process gateway"""
        
        self.queuesList["Config"].put(
            {
                "Subscribe/Unsubscribe": "subscribe",
                "Owner": MoveConfig.Owner.value,
                "msgID": MoveConfig.msgID.value,
                "To": {"receiver": "threadMove", "pipe": self.pipeSendconfig},
            }
        )
        
        self.queuesList["Config"].put(
            {
                "Subscribe/Unsubscribe": "subscribe",
                "Owner": EngineRun.Owner.value,
                "msgID": EngineRun.msgID.value,
                "To": {"receiver": "threadMove", "pipe": self.pipeSendstart},
            }
        )
        self.queuesList["Config"].put(
            {
                "Subscribe/Unsubscribe": "subscribe",
                "Owner": serialCamera.Owner.value,
                "msgID": serialCamera.msgID.value,
                "To": {"receiver": "threadMove", "pipe": self.pipeSendcamera_lf},
            }
        )
        
        self.queuesList["Config"].put(
            {
                "Subscribe/Unsubscribe": "subscribe",
                "Owner": ImuData.Owner.value,
                "msgID": ImuData.msgID.value,
                "To": {"receiver": "threadMove", "pipe": self.pipeIMUsend},
            }
        )   
        self.queuesList["Config"].put(
            {
                "Subscribe/Unsubscribe": "subscribe",
                "Owner": TrafficSign.Owner.value,
                "msgID": TrafficSign.msgID.value,
                "To": {"receiver": "threadMove", "pipe": self.pipesendTrSigns},
            }
        )

        self.queuesList["Config"].put(
            {
                "Subscribe/Unsubscribe": "subscribe",
                "Owner": Pedestrian.Owner.value,
                "msgID": Pedestrian.msgID.value,
                "To": {"receiver": "threadMove", "pipe": self.pipesendPed},
            }
        )    
        self.queuesList["Config"].put(
            {
                "Subscribe/Unsubscribe": "subscribe",
                "Owner": Path.Owner.value,
                "msgID": Path.msgID.value,
                "To": {"receiver": "threadMove", "pipe": self.pipeSendPathPlanning},
            }
        )

        self.queuesList["Config"].put(
            {
                "Subscribe/Unsubscribe": "subscribe",
                "Owner": InterDistance.Owner.value,
                "msgID": InterDistance.msgID.value,
                "To": {"receiver": "threadMove", "pipe": self.pipeSendInterDet},
            }
        )

        self.queuesList["Config"].put(
            {
                "Subscribe/Unsubscribe": "subscribe",
                "Owner": Pos.Owner.value,
                "msgID": Pos.msgID.value,
                "To": {"receiver": "threadMove", "pipe": self.pipeSendPos},
            }
        )


    # =============================== STOP ================================================
    def stop(self):
        super(threadMove, self).stop()

    # =============================== CONFIG ==============================================
    def Configs(self):
        """Function for receiving configs on the pipe."""
        while self.pipeRecvconfig.poll():
            message = self.pipeRecvconfig.recv()["value"]
            if (message["action"] == "autonomous"):
                self.autonomous = eval(message["value"])
            elif (message["action"] == "recording"):
                self.recording = eval(message["value"])
            elif (message["action"] == "speed"):
                self.speed = message["value"]
                print(self.speed)
            elif (message["action"] == "K_value"):
                self.K = message["value"]

    # =============================== START ===============================================
    def start(self):
        super(threadMove, self).start()
    
    # =============================== EMPTY RECEIVING PIPES ===============================
    
    def flush_all(self):
        for pipe in self.pipes:
            if pipe.poll():
                junk=pipe.recv()
                
    
    # ================================ RUN ================================================
    def run(self):
        "run function"
        
        # ========== Flags Needed ========== #
        sign_seen = False
        ped_seen = False
        intersection_seen = False
        intersection_searching = False
        
        time.sleep(0.5) #wait for initializations of the other processes

        self.queuesList[CalcPos.Queue.value].put( #send request to do position calculation
            {
                "Owner": CalcPos.Owner.value,
                "msgID": CalcPos.msgID.value,
                "msgType": CalcPos.msgType.value,
                "msgValue": True
            }   
        )

        viz = []        #to visualize IMU path - delete if forgoten :)

        
        while self._running:
            # ========== check if engine button is pressed ==========#
            if (self.pipeRecvstart.poll()):
                self.engine = self.pipeRecvstart.recv()["value"]
                if self.engine:
                    self.Configs()
                    self.flush_all()
                    for pipe in self.pipes:
                        pipe.send("ready")
                    print("running")
                    setSpeed(self.queuesList, self.speed)
                    logging.basicConfig(filename='example.log', level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
                    logging.info("-----------------------------NEW RUN---------------------------------------")
                    if self.recording:
                        start_recording(self.queuesList)
                        time.sleep(0.5)
                else:
                    print("stopped running")
                    if self.recording:
                        stop_recording(self.queuesList)
                self.pipeRecvstart.send("ready")
            
            # ========== If engine is on ==========#
            if self.engine:
                    
            # ========== If the car is autonomous ========== # 
                if self.autonomous:
                    try:
                        if self.piperecvTrSigns.poll():
                            sign=self.piperecvTrSigns.recv()["value"]
                            print("seen:", sign)
                            sign_seen = True
                            if (sign == "highway_entry" or sign == "highway_exit"):
                                sign_reaction(self.queuesList, sign)
                                self.flush_all()
                                #setSpeed(self.queuesList, self.speed)
                                for pipe in self.pipes:
                                    pipe.send("ready")
                                sign_seen = False
                            elif (sign == "Stop" or sign == "Crosswalk" or sign == "Parking"):
                                setSpeed(self.queuesList, 10)
                                sign_reaction(self.queuesList, sign) #comment for normal mode 
                                self.speed = 10
                            

                        
                        if self.piperecvPed.poll():
                            sign=self.piperecvPed.recv()["value"]
                            ped_seen = True
                            pedestrian_reaction(self.queuesList,ped_seen)
                            self.flush_all()
                            setSpeed(self.queuesList, self.speed)
                            for pipe in self.pipes:
                                pipe.send("ready")
                            sign_seen = False
                        
                        if self.pipeRecvInterDet.poll():
                            distance = self.pipeRecvInterDet.recv()["value"]
                            intersection_seen = True
                            setSpeed(self.queuesList, 10)
                            self.speed = 10
                            t = int(distance/(float(self.speed)/100))
                            start_time = time.time()
                            

                        '''   
                        if  not intersection_searching:
                            print("sent")
                            self.queuesList[Estimate.Queue.value].put( #send request to do intersection detection
                            {
                                "Owner": Estimate.Owner.value,
                                "msgID": Estimate.msgID.value,
                                "msgType": Estimate.msgType.value,
                                "msgValue": True
                            }
                            )
                            intersection_searching = True
                        '''
                        if intersection_seen:
                            if (time.time() - start_time) < (t+1):
                                if self.pipeRecvcamera_lf.poll():
                                    frame = self.pipeRecvcamera_lf.recv()
                                    image_data = base64.b64decode(frame["value"])
                                    img = np.frombuffer(image_data, dtype=np.uint8)
                                    image = cv2.imdecode(img, cv2.IMREAD_COLOR)
                                    angle = lf.followLane(image, self.K, self.speed)
                                    if angle is not None:
                                        angle = np.clip(angle, -25, 25)
                                        steer(self.queuesList, angle)
                                    self.pipeRecvcamera_lf.send("ready")
                            else:
                                brake(self.queuesList)
                                time.sleep(3)
                                # setSpeed(self.queuesList, 10)
                                # time.sleep(1)
                                # steer(self.queuesList, 25)
                                # time.sleep(5.5)
                                self.flush_all()
                                setSpeed(self.queuesList, self.speed)
                                for pipe in self.pipes:
                                    pipe.send("ready")
                                intersection_searching = False
                                intersection_seen = False


                        elif self.pipeRecvcamera_lf.poll():
                            frame = self.pipeRecvcamera_lf.recv()
                            image_data = base64.b64decode(frame["value"])
                            img = np.frombuffer(image_data, dtype=np.uint8)
                            image = cv2.imdecode(img, cv2.IMREAD_COLOR)
                            cv2.imwrite("test.jpg", image)
                            angle = lf.followLane(image, self.K, self.speed)
                            if angle is not None:
                                angle = np.clip(angle, -25, 25)
                                steer(self.queuesList, angle)
                            self.pipeRecvcamera_lf.send("ready")
                        
                    except:
                        logging.exception("Error in Thread Move", exc_info=True)
                        print("error")
                        if self.pipeRecvcamera_lf.poll():
                            self.pipeRecvcamera_lf.recv()
                        self.pipeRecvcamera_lf.send("ready")
                else:
                    # gostraight(self.pipeIMUrecv, self.queuesList, 180)
                    #======================INTERSECTION NAV=======================================
                    # xs = (0.83, 0)
                    # xf = (0, 0.76)
                    # phi = 89.9999
                    # if phi == 90:
                    #     phi = 89.9999
                    # dist1, angle, dist2 = intersection_navigation(xs, xf, phi)
                    # dist1 = 100*dist1
                    # dist2 = 100*dist2
                    # left_turn(self.pipeIMUrecv, self.queuesList, dist1, angle, dist2, self.speed)
                    # print(self.pipeIMUrecv.recv())
                    # self.pipeIMUrecv.send("ready")
                    


                    #========================LOCALIZATION==========================================

                    # steer(self.queuesList, 23)
                    # time.sleep(7)
                    # steer(self.queuesList, -22)
                    # time.sleep(10)
                    # setSpeed(self.queuesList, -10)
                    # time.sleep(2)
                    # x = [15,20,120,125]
                    # y = [19,19,56.5,56.5]
                    # x = np.asfarray(x)
                    # y = np.asfarray(y)
                    # park_the_car(x,y,self.queuesList)
                    
                    # try:
                        # if self.pipeRecvcamera_lf.poll():
                        #     frame = self.pipeRecvcamera_lf.recv()
                        #     image_data = base64.b64decode(frame["value"])
                        #     img = np.frombuffer(image_data, dtype=np.uint8)
                        #     image = cv2.imdecode(img, cv2.IMREAD_COLOR)
                        #     cv2.imwrite("test.jpg", image)
                        #     angle = lf.followLane(image, self.K, self.speed)
                        #     if angle is not None:
                        #         angle = np.clip(angle, -25, 25)
                        #         steer(self.queuesList, angle)
                        #     self.pipeRecvcamera_lf.send("ready")

                    #     if self.pipeRecvPos.poll():
                    #         coordinates = self.pipeRecvPos.recv()['value']
                    #         print(coordinates)

                    #         # Define the file name
                    #         file_name = "coordinates.txt"

                    #         # Open the file in write mode
                    #         with open(file_name, 'w') as file:
                    #             file.write(str(coordinates) + '\n')

                    #         self.pipeRecvPos.send("ready")

                    # except:
                    #     logging.exception("Error in Thread Move", exc_info=True)
                    #     print("error")
                    #     if self.pipeRecvcamera_lf.poll():
                    #         self.pipeRecvcamera_lf.recv()
                    #     self.pipeRecvcamera_lf.send("ready")
                    

                        
                    #========================CODE TO REQUEST PATH===================================

                    # self.queuesList[Calculate.Queue.value].put( #send request to do path calculation
                    #     {
                    #         "Owner": Calculate.Owner.value,
                    #         "msgID": Calculate.msgID.value,
                    #         "msgType": Calculate.msgType.value,
                    #         "msgValue": 472
                    #     }   
                    # )

                    # time.sleep(0.5)

                    # while self._running:
                    #     if self.pipeRecvPathPlanning.poll():
                    #         path = self.pipeRecvPathPlanning.recv()
                    #         print("Current path: ", path['value'])
                    #         self.pipeRecvPathPlanning.send("ready")
                    #         break

                    # if self.pipeRecvPos.poll():
                    #     coordinates = self.pipeRecvPos.recv()['value']
                    #     # print(coordinates)
                    #     viz.append(coordinates)

                    #     # Define the file name
                    #     file_name = "coordinates.txt"

                    #     with open(file_name, 'w') as file:
                    #         # Iterate over each tuple in coordinates
                    #         for coord in viz:
                    #             # Convert the tuple to a string and write it to the file followed by a newline
                    #             file.write(str(coord) + '\n')

                    #     self.pipeRecvPos.send("ready")

                    if self.pipeRecvPos.poll():
                        coordinates = self.pipeRecvPos.recv()['value']
                        # print(coordinates)

                        # Define the file name
                        file_path = 'coordinates.txt'
                        data_to_write = str(coordinates)

                        with open(file_path, 'r') as file:
                            lines = file.readlines()

                        empty_line_index = next((i for i, line in enumerate(lines) if line.strip() == ''), len(lines))

                        with open(file_path, 'a') as file:
                            if empty_line_index > 0:
                                file.write('\n')
                            file.write(data_to_write)


                        self.pipeRecvPos.send("ready")
