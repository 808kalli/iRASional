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

from src.templates.threadwithstop import ThreadWithStop
from twisted.internet import reactor
from src.data.CarsAndSemaphores.threads.udpListener import udpListener


class threadCarsAndSemaphores(ThreadWithStop):
    """Thread which will handle processCarsAndSemaphores functionalities

    Args:
        queuesList (dictionary of multiprocessing.queues.Queue): Dictionary of queues where the ID is the type of messages.
        listenPort (int, optional): Listening port. Defaults to 5007.
    """

    # ====================================== INIT ==========================================
    def __init__(self, queueList, listenPort=5007):
        super(threadCarsAndSemaphores, self).__init__()
        self.listenPort = listenPort
        self.queueList = queueList
        self.udp_factory = udpListener(self.queueList["General"])
        self.reactor = reactor
        self.reactor.listenUDP(self.listenPort, self.udp_factory)

    # ======================================= RUN ==========================================
    def run(self):
        self.reactor.run(installSignalHandlers=False)

<<<<<<< HEAD
=======

>>>>>>> 34965e969f6bceba984ba243fb9b98c0d90b4010
    # ====================================== STOP ==========================================
    def stop(self):
        self.reactor.stop()
        super(threadCarsAndSemaphores, self).stop()
