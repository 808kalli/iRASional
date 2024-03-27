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
<<<<<<< HEAD
=======

>>>>>>> 34965e969f6bceba984ba243fb9b98c0d90b4010
import threading
import time
import cv2
import numpy as np
import base64
import logging
<<<<<<< HEAD
from collections import deque
=======
>>>>>>> 34965e969f6bceba984ba243fb9b98c0d90b4010

from multiprocessing import Pipe
from src.utils.messages.allMessages import (
    ImuData, 
    serialCamera,
    EngineRun,
    TrafficSign,
<<<<<<< HEAD
    BatteryLvl,
    Pedestrian,
    Path,
    Calculate,
=======
    Pedestrian,
    Path,
>>>>>>> 34965e969f6bceba984ba243fb9b98c0d90b4010
    Estimate,
    InterDistance,
    MoveConfig,
    CurrentSpeed,
    Pos,
<<<<<<< HEAD
    Location
=======
    FrontDistance,
    Semaphores
>>>>>>> 34965e969f6bceba984ba243fb9b98c0d90b4010
)
from src.templates.threadwithstop import ThreadWithStop

from src.move.threads.movements.basic import setSpeed, steer, brake, start_recording, stop_recording
import src.move.threads.movements.lane_following as lf
from src.move.threads.movements.sign_reaction import sign_reaction
<<<<<<< HEAD
from src.move.threads.movements.pedestrian_reaction import pedestrian_reaction
from src.move.threads.movements.intersection import gostraight, right_turn, left_turn, intersection_navigation
=======
from src.move.threads.movements.intersection import gostraight, draw_trajectory, intersection_navigation

>>>>>>> 34965e969f6bceba984ba243fb9b98c0d90b4010
class threadMove(ThreadWithStop):
    """Thread which will handle the decision making.\n
    Args:
        pipeRecv: maybe
        pipeSend: maybe
<<<<<<< HEAD
        queuesList (dictionar of multiprocessing.queues.Queue): Dictionar of queues where the ID is the type of messages.
=======
        queuesList (dictionar of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
>>>>>>> 34965e969f6bceba984ba243fb9b98c0d90b4010
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
<<<<<<< HEAD
        self.K = 0.15 #0.12 or 0.15 maybe works
        self.speed = 15 #15 is default
        
        # flags
        self.autonomous = True
        self.recording = True
=======
        self.K = 0.09 #0.12 or 0.15 maybe works
        self.speed = 16 #15 is default
        
        # flags
        self.autonomous = True
        self.recording = False
>>>>>>> 34965e969f6bceba984ba243fb9b98c0d90b4010

        
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
<<<<<<< HEAD
=======
        pipeRecvsemaphores, pipeSendsemaphores = Pipe()
        self.pipeRecvsemaphores = pipeRecvsemaphores
        self.pipeSendsemaphores = pipeSendsemaphores
        pipeRecvfdist, pipeSendfdist = Pipe()
        self.pipes.append(self.pipeRecvsemaphores)
        self.pipeRecvfdist = pipeRecvfdist
        self.pipeSendfdist = pipeSendfdist
        self.pipes.append(self.pipeRecvfdist)
>>>>>>> 34965e969f6bceba984ba243fb9b98c0d90b4010
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
<<<<<<< HEAD
        for pipe in self.pipes:
            pipe.send("ready")
=======
        # for pipe in self.pipes:
        #     pipe.send("ready")
>>>>>>> 34965e969f6bceba984ba243fb9b98c0d90b4010

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
<<<<<<< HEAD
=======
        
        self.queuesList["Config"].put(
            {
                "Subscribe/Unsubscribe": "subscribe",
                "Owner": FrontDistance.Owner.value,
                "msgID": FrontDistance.msgID.value,
                "To": {"receiver": "threadMove", "pipe": self.pipeSendfdist},
            }
        )
        
        self.queuesList["Config"].put(
            {
                "Subscribe/Unsubscribe": "subscribe",
                "Owner": Semaphores.Owner.value,
                "msgID": Semaphores.msgID.value,
                "To": {"receiver": "threadMove", "pipe": self.pipeSendsemaphores},
            }
        )
>>>>>>> 34965e969f6bceba984ba243fb9b98c0d90b4010


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
<<<<<<< HEAD
                self.K = message["value"]
=======
                self.K = float(message["value"])
>>>>>>> 34965e969f6bceba984ba243fb9b98c0d90b4010

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
        
<<<<<<< HEAD
        # ========== Flags Needed ========== #
        sign_seen = False
        ped_seen = False
        intersection_seen = False
        intersection_searching = False
        
        time.sleep(0.5) #wait for initializations of the other processes

        viz = []

        
        while self._running:
            # ========== check if engine button is pressed ==========#
=======
        " Here go the necessary initializations before starting the loop of the main flow"
        # ========== Flags Needed ========== #
        sign_seen = False
        intersection_seen = False
        intersection_searching = False
        reduced_speed = False
        parking_found = False
        directions = []
        
        A = 0.4
        distance=0
        EAM=20
        #start of parking spot
        parking_time=0
        count = 0
        
        
        time.sleep(0.5) #wait for initializations of the other processes    
        while self._running:
            
            """"
            If the engine button is pressed: 
            if engine button is on start running the car
            if the engine button is off stop the car
            """
            
            """
            TO DO
            Inside this if should start everything we need for the rest of the main flow (for example the path planning)
            """
            
>>>>>>> 34965e969f6bceba984ba243fb9b98c0d90b4010
            if (self.pipeRecvstart.poll()):
                self.engine = self.pipeRecvstart.recv()["value"]
                if self.engine:
                    self.Configs()
                    self.flush_all()
                    for pipe in self.pipes:
                        pipe.send("ready")
<<<<<<< HEAD
                    print("running")
                    setSpeed(self.queuesList, self.speed)
                    logging.basicConfig(filename='example.log', level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
                    logging.info("-----------------------------NEW RUN---------------------------------------")
=======
                    intersection_searching = False
                        
                    # here should go the path planning 
                    
                    directions = ["LEFT", "STRAIGHT", "END"]
                    
                    print("running")
                    setSpeed(self.queuesList, self.speed)
                    # logging.basicConfig(filename='example.log', level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
                    # logging.info("-----------------------------NEW RUN---------------------------------------")
>>>>>>> 34965e969f6bceba984ba243fb9b98c0d90b4010
                    if self.recording:
                        start_recording(self.queuesList)
                        time.sleep(0.5)
                else:
                    print("stopped running")
                    if self.recording:
                        stop_recording(self.queuesList)
<<<<<<< HEAD
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
=======
                self.pipeRecvstart.send("ready")    
         
            
            if self.engine:
                
                "If the car runs autonomous"
                
                """
                TO DO
                in the exception check if we need to send ready to every pipe (or make any changes)
                """
                if self.autonomous:
                    
                    # ==================== First just do lanefollowing (important) ==================== #
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

                    # ==================== Then do the checks ==================== #
                    
                    # -------------------- Check number 1: intersection detection -------------------- #
                    if  ((not intersection_searching) and (not intersection_seen)):
                        self.queuesList[Estimate.Queue.value].put( #send request to do intersection detection
                        {
                            "Owner": Estimate.Owner.value,
                            "msgID": Estimate.msgID.value,
                            "msgType": Estimate.msgType.value,
                            "msgValue": True
                        }
                        )
                    
                        intersection_searching = True
                    
                    if self.pipeRecvInterDet.poll():
                        distance = self.pipeRecvInterDet.recv()["value"]
                        intersection_seen = True
                        intersection_searching = False
                        print("intersection in distance = ", distance)
                        t = int(distance/(float(self.speed)/100))
                        start_time = time.time()
                        
                    # -------------------- Check number 2: traffic signs -------------------- #
                    if self.piperecvTrSigns.poll():
                        sign=self.piperecvTrSigns.recv()["value"]
                        sign_seen = True
                        
                        
                    # -------------------- Check number 3: pedestrian -------------------- #
                    if self.piperecvPed.poll():
                        Pedestrian =self.piperecvPed.recv()["value"]
                        self.piperecvPed.send("ready")
                        if(not intersection_seen): #to avoid stopping before crosswalk
                            print("seen pedestrian")
                            brake(self.queuesList)
                            time.sleep(2)
                            
                            if not self.piperecvPed.poll():
                                setSpeed(self.queuesList, self.speed)
                    
                    # ==================== Use the flags to find if we need a reaction ==================== #
                    
                    if intersection_seen:
                        
                        if (time.time() - start_time <= t + 0.5):
                            if(sign_seen and sign == "Crosswalk" and (not reduced_speed)):
                                print("seen sign:", sign)
                                setSpeed(self.queuesList, 10)
                                t = t + t/3
                                reduced_speed = True
                            continue
                        
                        if (sign_seen == False):
                            current = directions.pop(0)
                            intersection_navigation(current, self.pipeIMUrecv, self.queuesList)
                       
                        elif (sign == "Priority" or sign == "Stop"):
                            print("seen sign:", sign)
                            sign_reaction(self.queuesList, sign)
                            setSpeed(self.queuesList, self.speed)
                            current = directions.pop(0)
                            intersection_navigation(current, self.pipeIMUrecv, self.queuesList)
                        
                        elif (sign == "Crosswalk"):
                            sign_reaction(self.queuesList, sign, self.piperecvPed)
                            setSpeed(self.queuesList, self.speed)                           
                            gostraight(self.pipeIMUrecv, self.queuesList, int(70/int(self.speed)))
                        
                        elif (sign == "Parking"):
                            print("seen sign:", sign)
                            # find_parking()
                            if (not parking_found):
                                print("here")
                                distance = float(self.pipeRecvfdist.recv()["value"])
                                self.pipeRecvfdist.send("ready")
                                print("distance =", distance)
                                if(EAM-distance<-15):
                                    if(count==0):
                                        parking_time=time.time()
                                    count+=1
                                    print("count =", count)
                                else:
                                    EAM=(1-A)*EAM+A*distance
                                    print("EAM =", EAM)
                                    count=0
                                if(count>9):
                                    dt = time.time() - parking_time
                                    if(dt>3.5):
                                        parking_found = True
                                        parking_time2 = time.time()
                                        #start parking
                                    else:
                                        parking_time=time.time()
                                continue
                            else:
                                t0 = (115/17) - dt
                                if (time.time() - parking_time2 < t0):
                                    continue
                                brake(self.queuesList)
                                time.sleep(2)
                                sign_reaction(self.queuesList, "Parking")
                                setSpeed(self.queuesList, self.speed)
                        else:
                            print("seen sign:", sign)
                            sign_reaction(self.queuesList, sign)
                            
                        parking_found = False
                        sign_seen = False
                        intersection_seen = False
                        reduced_speed = False
                        self.flush_all()
                        for pipe in self.pipes:
                            pipe.send("ready")

                        
                # -------->> EVERYTHING AFTER "else" IS FOR TESTING
                else:
                    if self.pipeIMUrecv.poll():
                        print(self.pipeIMUrecv.recv())
                        self.pipeIMUrecv.send("ready")
                    pass
                    
                    # ================ Semaphores Testing ==================================#
                    # while(True):
                    #     Semaphore = (self.pipeRecvsemaphores.recv())
                    #     print(Semaphore)
                    #     if(Semaphore["id"]==2):
                    #         print(Semaphore["value"]["state"])
                    #     self.pipeRecvsemaphores.send("ready")
                    # pass


                    #========================LOCALIZATION + LANE FOLLOWING==========================================
                    
                    try:
                        if self.pipeRecvcamera_lf.poll():
>>>>>>> 34965e969f6bceba984ba243fb9b98c0d90b4010
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
<<<<<<< HEAD
                        
=======

                            self.queuesList[CurrentSpeed.Queue.value].put( #send current velocity to do position calculation
                                {
                                    "Owner": CurrentSpeed.Owner.value,
                                    "msgID": CurrentSpeed.msgID.value,
                                    "msgType": CurrentSpeed.msgType.value,
                                    "msgValue": 0.15
                                }   
                            )

                        if self.pipeRecvPos.poll():
                            coordinates = self.pipeRecvPos.recv()['value']
                            coordinates = (coordinates[0], coordinates[1])
                            # print(coordinates)

                            # Graph trajectory
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

>>>>>>> 34965e969f6bceba984ba243fb9b98c0d90b4010
                    except:
                        logging.exception("Error in Thread Move", exc_info=True)
                        print("error")
                        if self.pipeRecvcamera_lf.poll():
                            self.pipeRecvcamera_lf.recv()
                        self.pipeRecvcamera_lf.send("ready")
<<<<<<< HEAD
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

                    # try:
                    #     if self.pipeRecvcamera_lf.poll():
                    #         frame = self.pipeRecvcamera_lf.recv()
                    #         image_data = base64.b64decode(frame["value"])
                    #         img = np.frombuffer(image_data, dtype=np.uint8)
                    #         image = cv2.imdecode(img, cv2.IMREAD_COLOR)
                    #         cv2.imwrite("test.jpg", image)
                    #         angle = lf.followLane(image, self.K, self.speed)
                    #         if angle is not None:
                    #             angle = np.clip(angle, -25, 25)
                    #             steer(self.queuesList, angle)
                    #         self.pipeRecvcamera_lf.send("ready")

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
=======
>>>>>>> 34965e969f6bceba984ba243fb9b98c0d90b4010
                    

                        
                    #========================CODE TO REQUEST PATH===================================

                    # self.queuesList[Calculate.Queue.value].put( #send request to do path calculation
                    #     {
                    #         "Owner": Calculate.Owner.value,
                    #         "msgID": Calculate.msgID.value,
                    #         "msgType": Calculate.msgType.value,
<<<<<<< HEAD
                    #         "msgValue": 472
=======
                    #         "msgValue": 472       #target node
>>>>>>> 34965e969f6bceba984ba243fb9b98c0d90b4010
                    #     }   
                    # )

                    # time.sleep(0.5)

                    # while self._running:
                    #     if self.pipeRecvPathPlanning.poll():
                    #         path = self.pipeRecvPathPlanning.recv()
                    #         print("Current path: ", path['value'])
                    #         self.pipeRecvPathPlanning.send("ready")
<<<<<<< HEAD
                    #         break

                    self.queuesList[CurrentSpeed.Queue.value].put( #send request to do position calculation
                        {
                            "Owner": CurrentSpeed.Owner.value,
                            "msgID": CurrentSpeed.msgID.value,
                            "msgType": CurrentSpeed.msgType.value,
                            "msgValue": 0.15    # m/s
                        }   
                    )

                    if self.pipeRecvPos.poll():
                        coordinates, (pos_x, pos_y) = self.pipeRecvPos.recv()['value']
                        # print(coordinates)
                        viz.append((coordinates, (pos_x, pos_y)))

                        # Define the file name
                        file_name = "coordinates.txt"

                        with open(file_name, 'w') as file:
                            # Iterate over each tuple in coordinates
                            for coord in viz:
                                # Convert the tuple to a string and write it to the file followed by a newline
                                file.write(str(coord) + '\n')

                        self.pipeRecvPos.send("ready")
=======
                    #         break
>>>>>>> 34965e969f6bceba984ba243fb9b98c0d90b4010
