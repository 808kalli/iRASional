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
from twisted.internet import protocol
import json
from src.utils.messages.allMessages import (
    EngineRun,
    SpeedMotor,
    MoveConfig,
    SteerMotor,
    BatteryLvl,
    LaneError,
    Brake
)


# One class is generated for each new connection
class SingleConnection(protocol.Protocol):
    # ================================= CONNECTION MADE =====+================================
    def connectionMade(self):
        peer = self.transport.getPeer()
        self.factory.connectiondata = peer.host + ":" + str(peer.port)
        self.factory.connection = self
        self.connected = False
        print("Attempting connection by :", self.factory.connectiondata)

    # ================================= CONNECTION LOST ======================================
    def connectionLost(self, reason):
        print("Connection lost with ", self.factory.connectiondata, " due to: ", reason)
        self.factory.isConnected = False
        self.factory.isConnected = False
        self.factory.connectiondata = None
        self.factory.connection = None

    # ================================== DATA RECEIVED =======================================
    def dataReceived(self, data):
        if self.factory.isConnected == False:
            pswd = data.decode()
            if pswd == "Ala-Bala":
                self.factory.isConnected = True
                print(
                    "Connected with ", self.factory.connectiondata, " : ", data.decode()
                )
            else:
                print(
                    "Connection attempted failed with incorrect password ",
                    self.factory.connectiondata,
                )
                self.factory.isConnected = False
                self.factory.connectiondata = None
                self.transport.loseConnection()
                self.factory.connection = None
        else:
            dataJSON = json.loads(data.decode())
            # print(dataJSON)
            if dataJSON["action"] == "startEngine":
                self.factory.queues[EngineRun.Queue.value].put(
                    {
                        "Owner": EngineRun.Owner.value,
                        "msgID": EngineRun.msgID.value,
                        "msgType": EngineRun.msgType.value,
                        "msgValue": dataJSON["value"],
                    }
                )
            elif (dataJSON["action"] == "speed" or dataJSON["action"] == "K_value" or dataJSON["action"] == "recording" or dataJSON["action"] == "autonomous"):
                self.factory.queues[MoveConfig.Queue.value].put(
                    {
                        "Owner": MoveConfig.Owner.value,
                        "msgID": MoveConfig.msgID.value,
                        "msgType": MoveConfig.msgType.value,
                        "msgValue": dataJSON
                    }
                )
            # print("Received from", self.factory.connectiondata, " : ", data.decode())

    # ===================================== SEND DATA ==========================================
    def send_data(self, messageValue, messageType, messageOwner, messageId):
        """This function will send firstly an encoded message as an int represented in one byte after that it will send the lenght of the message and the message."""
        self.transport.write(
            self.factory.encoder[(messageType, messageOwner, messageId)].to_bytes(
                1, byteorder="big"
            )
        )
        self.transport.write(
            len(messageValue.encode("utf-8")).to_bytes(4, byteorder="big")
        )  # send size of image
        self.transport.write(messageValue.encode("utf-8"))  # send image data


from src.utils.messages.allMessages import (
    mainCamera,
    EnableButton,
    SignalRunning,
    Location,
    Signal,
)


# The server itself. Creates a new Protocol for each new connection and has the info for all of them.
class FactoryDealer(protocol.Factory):
    # ======================================= INIT =============================================
    def __init__(self, queues):
        self.connection = None
        self.isConnected = False
        self.connectiondata = None
        self.queues = queues
        self.encoder = {
            (
                EnableButton.msgType.value,
                EnableButton.Owner.value,
                EnableButton.msgID.value,
            ): 1,
            (
                SignalRunning.msgType.value,
                SignalRunning.Owner.value,
                SignalRunning.msgID.value,
            ): 2,
            (
                Location.msgType.value,
                Location.Owner.value,
                Location.msgID.value,
            ): 3,
            (
                Signal.msgType.value,
                Signal.Owner.value,
                Signal.msgID.value,
            ): 4,
            (
                mainCamera.msgType.value,
                mainCamera.Owner.value,
                mainCamera.msgID.value,
            ): 5,
            # (
            #     Alerts.msgType.value,
            #     Alerts.Owner.value,
            #     Alerts.msgID.value,
            # ): 6,
            # (
            #     SpeedMotor.msgType.value,
            #     SpeedMotor.Owner.value,
            #     SpeedMotor.msgID.value,
            # ): 7,
            
            # (
            #     Brake.msgType.value,
            #     Brake.Owner.value,
            #     Brake.msgID.value,
            # ): 7,
            
            # (
            #     BatteryLvl.msgType.value,
            #     BatteryLvl.Owner.value,
            #     BatteryLvl.msgID.value,
            # ): 8,
            # (
            #     LaneError.msgType.value,
            #     LaneError.Owner.value,
            #     LaneError.msgID.value,
            # ): 9,
            # (
            #     SteerMotor.msgType.value,
            #     SteerMotor.Owner.value,
            #     SteerMotor.msgID.value,
            # ): 10,
                
        }

    def send_data_to_client(self, messageValue, messageType, messageOwner, messageId):
        """This function will try to send the information only if there is a connection between DashBoard and raspberry PI."""
        if self.isConnected == True:
            self.connection.send_data(
                messageValue, messageType, messageOwner, messageId
            )
        else:
            print("Client not connected")

    # ================================== BUILD PROTOCOL ========================================
    def buildProtocol(self, addr):
        conn = SingleConnection()
        conn.factory = self
        return conn

    # =================================== AUXILIARY ============================================
    def doStart(self):
        print("Start factory")

    def doStop(self):
        print("Stop factory")
