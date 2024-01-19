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
from collections import deque

from multiprocessing import Pipe
from src.utils.messages.allMessages import (
mainCamera
)
from src.templates.threadwithstop import ThreadWithStop
from src.traffic_signs.TrafficSigns import TrafficSignDetection


class threadTraffic_Signs(ThreadWithStop):
    """Thread which will detect traffic signs.\n
    Args:
        pipeRecv: maybe
        pipeSend: maybe
        queuesList (dictionar of multiprocessing.queues.Queue): Dictionar of queues where the ID is the type of messages.
        logger (logging object): Made for debugging.
        debugger (bool): A flag for debugging.
    """

    # ================================ INIT ===============================================
    def __init__(self, queuesList, logger, debugger):
        super(threadTraffic_Signs, self).__init__()
        self.queuesList = queuesList
        self.logger = logger
        self.debugger = debugger
        pipeRecvcamera_signs, pipeSendcamera_signs = Pipe()     
        self.pipeRecvcamera_signs = pipeRecvcamera_signs
        self.pipeSendcamera_signs = pipeSendcamera_signs
        self.subscribe()
        self.pipeRecvcamera_signs.send("ready")

    def subscribe(self):
        """Subscribe function. In this function we make all the required subscribe to process gateway"""
        self.queuesList["Config"].put(
            {
                "Subscribe/Unsubscribe": "subscribe",
                "Owner": mainCamera.Owner.value,
                "msgID": mainCamera.msgID.value,
                "To": {"receiver": "threadTrafficSigns", "pipe": self.pipeSendcamera_signs},
            }
        )


    # =============================== STOP ================================================
    def stop(self):
        super(threadTraffic_Signs, self).stop()

    # =============================== CONFIG ==============================================
    #def Configs(self):
    #    """Callback function for receiving configs on the pipe."""

    # =============================== START ===============================================
    def start(self):
        super(threadTraffic_Signs, self).start()

    # ================================ RUN ================================================
    def run(self):
        while self._running:
            #print("working")
            if self.pipeRecvcamera_signs.poll():
                frame = self.pipeRecvcamera_signs.recv()
                image_data = base64.b64decode(frame["value"])
                img = np.frombuffer(image_data, dtype=np.uint8)
                image = cv2.imdecode(img, cv2.IMREAD_COLOR)
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                cv2.imwrite("received.jpg",image)
                detected, sign = TrafficSignDetection(image)
                print("detected:", detected, "sign", sign)
                self.pipeRecvcamera_signs.send("ready")
