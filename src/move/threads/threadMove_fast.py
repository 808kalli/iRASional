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
mainCamera
)
from src.templates.threadwithstop import ThreadWithStop

from src.move.threads.movements.basic import setSpeed, steer, brake, start_recording
import src.move.threads.movements.lane_following as lf
import src.move.threads.movements.lane_following_old as lf_old

class threadMove_fast(ThreadWithStop):
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
        super(threadMove_fast, self).__init__()
        self.queuesList = queuesList
        self.logger = logger
        self.debugger = debugger
        self.pipes = list()
        pipeRecvcamera_lf, pipeSendcamera_lf = Pipe()
        pipeTESTrecv, pipeTESTsend = Pipe()
        self.pipeTESTrecv = pipeTESTrecv
        self.pipes.append(self.pipeTESTrecv)
        self.pipeTESTsend = pipeTESTsend       
        self.pipeRecvcamera_lf = pipeRecvcamera_lf
        self.pipes.append(self.pipeRecvcamera_lf)
        self.pipeSendcamera_lf = pipeSendcamera_lf
        self.subscribe()
        for pipe in self.pipes:
            pipe.send("ready")

    def subscribe(self):
        """Subscribe function. In this function we make all the required subscribe to process gateway"""
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
                "Owner": BatteryLvl.Owner.value,
                "msgID": BatteryLvl.msgID.value,
                "To": {"receiver": "threadMove", "pipe": self.pipeTESTsend},
            }
        )


    # =============================== STOP ================================================
    def stop(self):
        brake(self.queuesList)
        
        self.queuesList[EngineRun.Queue.value].put(
        {
        "Owner": EngineRun.Owner.value,
        "msgID": EngineRun.msgID.value,
        "msgType": EngineRun.msgType.value,
        "msgValue": False
        }
        )
        super(threadMove_fast, self).stop()

    # =============================== CONFIG ==============================================
    #def Configs(self):
    #    """Callback function for receiving configs on the pipe."""

    # =============================== START ===============================================
    def start(self):
        super(threadMove_fast, self).start()
    
    # =============================== EMPTY RECEIVING PIPES ===============================
    
    def flush_all(self):
        for pipe in self.pipes:
            while pipe.poll():
                junk=pipe.recv()

    # ================================ RUN ================================================
    def run(self):
        "run function"
        self.queuesList[EngineRun.Queue.value].put(
        {
        "Owner": EngineRun.Owner.value,
        "msgID": EngineRun.msgID.value,
        "msgType": EngineRun.msgType.value,
        "msgValue": True
        }   
        )
        # Setting the parameters for lane following
        K = 0.2
        speed = 20 #15 is default
        
        
        time.sleep(0.5)
        # start_recording(self.queuesList)
        time.sleep(0.5)
        brake(self.queuesList)
        time.sleep(0.5)
        print("running")
        setSpeed(self.queuesList)
        logging.basicConfig(filename='example.log', level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
        logging.info("-----------------------------NEW RUN---------------------------------------")
        while self._running:
            try:
                # uncomment to get IMU data
                
                # if self.pipeTESTrecv.poll():
                #     data = self.pipeTESTrecv.recv()
                #     print("IMU data", data["value"])
                #     self.pipeTESTrecv.send("ready")

                # uncomment to run lane following
                
                if self.pipeRecvcamera_lf.poll():
                    frame = self.pipeRecvcamera_lf.recv()
                    image_data = base64.b64decode(frame["value"])
                    img = np.frombuffer(image_data, dtype=np.uint8)
                    image = cv2.imdecode(img, cv2.IMREAD_COLOR)
                    cv2.imwrite("test.jpg", image)
                    angle = lf.followLane(image, K, speed)
                    angle = np.clip(angle, -22, 22)
                    # print("angle", angle)
                    steer(self.queuesList, angle)
                    self.pipeRecvcamera_lf.send("ready")
            except:
                logging.exception("Error in Thread Move - Fast Version", exc_info=False)