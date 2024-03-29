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

from multiprocessing import Pipe
from src.utils.messages.allMessages import (
    ImuData, 
    serialCamera,
    EngineRun,
    TrafficSign,
    Pedestrian,
    Path,
    Estimate,
    InterDistance,
    MoveConfig,
    Pos,
    FrontDistance,
    Semaphores,
    CurrentSpeed,
    Calculate,
    SignsSearching
)
from src.templates.threadwithstop import ThreadWithStop

from src.move.threads.movements.basic import setSpeed, steer, brake, start_recording, stop_recording
import src.move.threads.movements.lane_following as lf
from src.move.threads.movements.sign_reaction import sign_reaction
from src.move.threads.movements.intersection import gostraight, draw_trajectory, intersection_navigation
from src.move.threads.movements.roundabout import draw_roundabout_trajectory
from src.move.threads.movements.tailing import tail

class threadMove(ThreadWithStop):
    """Thread which will handle the decision making.\n
    Args:
        pipeRecv: maybe
        pipeSend: maybe
        queuesList (dictionar of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
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
        self.K = 0.09 #0.12 or 0.15 maybe works
        self.speed = 16 #15 is default
        
        # flags
        self.autonomous = True
        self.recording = False
        self.functional = False #allows the robot to do anything if current node is on the calculated path


        self.current_node = None #initialize current node
        self.coordinates = None
        self.path = []
        self.directions = []

        #critical points
        self.stop_pos = [*range(44,50,1),*range(8,15,1), *range(74,79,1)]
        self.prioriy_pos = [*range(44,50,1),*range(8,15,1), *range(74,79,1)]
        self.parking_pos =[*range(27,37,1), *range(66,75,1)]
        self.crosswalk_pos =[*range(14,20,1), *range(51,60,1)]
        self.intersection = [49,8,78]

        
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
        pipeRecvsemaphores, pipeSendsemaphores = Pipe()
        self.pipeRecvsemaphores = pipeRecvsemaphores
        self.pipeSendsemaphores = pipeSendsemaphores
        pipeRecvfdist, pipeSendfdist = Pipe()
        self.pipes.append(self.pipeRecvsemaphores)
        self.pipeRecvfdist = pipeRecvfdist
        self.pipeSendfdist = pipeSendfdist
        self.pipes.append(self.pipeRecvfdist)
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
        # for pipe in self.pipes:
        #     pipe.send("ready")

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
                self.K = float(message["value"])

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
        
        " Here go the necessary initializations before starting the loop of the main flow"
        
        #initialize current node
        same_node_counter = 0
        initialize_pos = False
        initialization = True #for first loop only

        directions = []

        # ========== Flags Needed ========== #
        intersection_searching = False
        intersection_seen = False

        sign_seen = False

        reduced_speed = False
        parking_found = False
        tailing = False
        
        lane_offset = 0
        
        A = 0.4
        distance=0
        EAM=20
        #start of parking spot
        parking_time=0
        count = 0
        counttail = 0
        
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
            while initialize_pos == False:
                self.queuesList[CurrentSpeed.Queue.value].put( #send current velocity to do position calculation
                    {
                        "Owner": CurrentSpeed.Owner.value,
                        "msgID": CurrentSpeed.msgID.value,
                        "msgType": CurrentSpeed.msgType.value,
                        "msgValue": 0.0
                    }   
                )

                if self.pipeRecvPos.poll():
                    (coordinates, node) = self.pipeRecvPos.recv()['value']
                    
                    if node == self.current_node:
                        same_node_counter += 1
                    else:
                        same_node_counter = 0  # Reset the counter if the node changes
                        self.current_node = node  # Update the current node
                    
                    if same_node_counter == 5:
                        initialize_pos = True

            if (initialize_pos == True and initialization == True):
                self.queuesList[Calculate.Queue.value].put( #send request to do path calculation
                    {
                        "Owner": Calculate.Owner.value,
                        "msgID": Calculate.msgID.value,
                        "msgType": Calculate.msgType.value,
                        "msgValue": self.current_node       #source node
                    }   
                )    

                print("Starting node: ", self.current_node)

                if self.pipeRecvPathPlanning.poll():    #recieve path to follow
                    (self.path, self.directions) = self.pipeRecvPathPlanning.recv()['value']
                    self.pipeRecvPathPlanning.send("ready")
                
                initialization = False      #position initialization is over


            '''==================================== WE HAVE A PATH TO FOLLOW ========================================='''
            '''
            Run everything and check if you are in the right node while checking
            '''
            if (self.pipeRecvstart.poll()):
                self.engine = self.pipeRecvstart.recv()["value"]
                if self.engine:
                    self.Configs()
                    self.flush_all()
                    for pipe in self.pipes:
                        pipe.send("ready")
                    intersection_searching = False
                                            
                    directions = self.directions
                    
                    print("running")
                    setSpeed(self.queuesList, self.speed)
                    # logging.basicConfig(filename='example.log', level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
                    # logging.info("-----------------------------NEW RUN---------------------------------------")
                    if self.recording:
                        start_recording(self.queuesList)
                        time.sleep(0.5)
                else:
                    print("stopped running")
                    if self.recording:
                        stop_recording(self.queuesList)
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
                        angle, lane_offset = lf.followLane(image, self.K, self.speed)
                        if angle is not None:
                            angle, offset = np.clip(angle, -25, 25)
                            steer(self.queuesList, angle)

                        self.pipeRecvcamera_lf.send("ready")

                    # ==================== Then do the checks ==================== #
                        
                    # ------------------- Check number 0: localization ----------------------#
                    if self.pipeRecvPos.poll():
                        (coordinates, node) = self.pipeRecvPos.recv()['value']

                        self.coordinates = (coordinates[0], coordinates[1])
                        self.current_node = node

                        print("Current node: ", node, " at coordinates: ", coordinates)

                        self.pipeRecvPos.send("ready")
                    
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
                        if self.current_node in self.stop_pos | self.prioriy_pos | self.parking_pos: #check if in correct nodes for intersection seen
                            distance = self.pipeRecvInterDet.recv()["value"]
                            intersection_seen = True
                            intersection_searching = False
                            print("intersection in distance = ", distance)
                            t = int(distance/(float(self.speed)/100))
                            start_time = time.time()
                        
                    # -------------------- Check number 2: traffic signs -------------------- #
                    if self.piperecvTrSigns.poll():
                        if self.current_node in self.stop_pos | self.prioriy_pos | self.parking_pos | self.crosswalk_pos: #see sign if in node with sign
                            sign=self.piperecvTrSigns.recv()["value"]
                            sign_seen = True
                            
                        
                    # -------------------- Check number 3: pedestrian -------------------- #
                    if self.piperecvPed.poll(): #check for pedestrian anywhere
                        Pedestrian =self.piperecvPed.recv()["value"]
                        self.piperecvPed.send("ready")
                        if(not intersection_seen): #to avoid stopping before crosswalk
                            print("seen pedestrian")
                            brake(self.queuesList)
                            time.sleep(2)

                            if not self.piperecvPed.poll():
                                setSpeed(self.queuesList, self.speed)
                    
                    # ------------------- Check number 5: tailing conditions ----------------------#
                    if self.pipeRecvfdist.poll(): #check for tailing not available yet...
                        front_dist = self.pipeRecvfdist.recv()["value"]
                        self.pipeRecvfdist.send("ready")
                        if(front_dist < 80):
                            counttail = counttail + 1
                        else:
                            counttail = 0
                        if(counttail>=4):
                            steer(self.queuesList, 0)
                            tail(self.pipeRecvfdist, self.queuesList, self.pipeRecvcamera_lf, self.K, 100 )

                    # ==================== Use the flags to find if we need a reaction ==================== #
                    
                    if intersection_seen:   #made check earlier
                        
                        if (time.time() - start_time <= t + 0.5):
                            if(sign_seen and sign == "Crosswalk" and (not reduced_speed)):
                                print("seen sign:", sign)
                                setSpeed(self.queuesList, 10)
                                t = t + t/3
                                reduced_speed = True
                            continue
                        
                        if (sign_seen == False):
                            current = directions.pop(0)
                            brake(self.queuesList)
                            time.sleep(2)
                            setSpeed(self.queuesList, self.speed)
                            intersection_navigation(current, self.pipeIMUrecv, self.queuesList, lane_offset)
                       
                        elif (sign == "Priority" or sign == "Stop"):
                            if self.current_node in self.stop_pos | self.prioriy_pos:
                                print("Seen sign:", sign)
                                sign_reaction(self.queuesList, sign)
                                setSpeed(self.queuesList, self.speed)
                                current = directions.pop(0)
                                intersection_navigation(current, self.pipeIMUrecv, self.queuesList, lane_offset)
                        
                        elif (sign == "Crosswalk"):
                            if self.current_node in self.crosswalk_pos:
                                sign_reaction(self.queuesList, sign, self.piperecvPed)
                                setSpeed(self.queuesList, self.speed)
                                gostraight(self.pipeIMUrecv, self.queuesList, int(70/int(self.speed)))
                        
                        elif (sign == "Parking"):
                            if self.current_node in self.parking_pos:
                                print("seen sign:", sign)
                                # find_parking()
                                if (not parking_found):
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
                        else:   #for rest of signs (e.g highway), to be completed later...
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
                    angles, distances = draw_roundabout_trajectory(0, 0, 0)
                    angle=0
                    i = 0
                    for dist in distances:
                        t = dist / 15.0
                        if (angles[i] > 0 and angle < 0):
                            angle = -7
                        elif (angles[i] < 0 and angle > 20):
                            angle = 15
                        else:
                            angle = angle + angles[i]
                        steer(self.queuesList, angle)
                        i = i + 1
                        time.sleep(t)
                    brake(self.queuesList)
                    break
                    # if self.pipeIMUrecv.poll():
                    #     print(self.pipeIMUrecv.recv())
                    #     self.pipeIMUrecv.send("ready")
                    # pass
                    
                    # ================ Semaphores Testing ==================================#
                    # while(True):
                    #     Semaphore = (self.pipeRecvsemaphores.recv())
                    #     print(Semaphore)
                    #     if(Semaphore["id"]==2):
                    #         print(Semaphore["value"]["state"])
                    #     self.pipeRecvsemaphores.send("ready")
                    # pass


                    #========================LOCALIZATION + LANE FOLLOWING==========================================
                    
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

                            # self.queuesList[CurrentSpeed.Queue.value].put( #send current velocity to do position calculation
                            #     {
                            #         "Owner": CurrentSpeed.Owner.value,
                            #         "msgID": CurrentSpeed.msgID.value,
                            #         "msgType": CurrentSpeed.msgType.value,
                            #         "msgValue": 0.15
                            #     }   
                            # )

                    #     if self.pipeRecvPos.poll():
                    #         (coordinates, node) = self.pipeRecvPos.recv()['value']
                    #         print(node)
                    #         coordinates = (coordinates[0], coordinates[1])
                    #         # print(coordinates)

                    #         # Graph trajectory
                    #         file_path = 'coordinates.txt'
                    #         data_to_write = str(coordinates)

                    #         with open(file_path, 'r') as file:
                    #             lines = file.readlines()

                    #         empty_line_index = next((i for i, line in enumerate(lines) if line.strip() == ''), len(lines))

                    #         with open(file_path, 'a') as file:
                    #             if empty_line_index > 0:
                    #                 file.write('\n')
                    #             file.write(data_to_write)


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
                    #         "msgValue": 472       #target node
                    #     }   
                    # )

                    # time.sleep(0.5)

                    # while self._running:
                    #     if self.pipeRecvPathPlanning.poll():
                    #         path = self.pipeRecvPathPlanning.recv()
                    #         print("Current path: ", path['value'])
                    #         self.pipeRecvPathPlanning.send("ready")
                    #         break