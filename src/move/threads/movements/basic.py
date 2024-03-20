from src.utils.messages.allMessages import (
SpeedMotor,
SteerMotor,
Brake,
Record
)

def setSpeed(queuesList, speed=15):
    # print("#----- setting speed -----#")
    # print("speed = ", speed)
    queuesList[SpeedMotor.Queue.value].put(
    {
        "Owner": SpeedMotor.Owner.value,
        "msgID": SpeedMotor.msgID.value,
        "msgType": SpeedMotor.msgType.value,
        "msgValue": speed
    }
    )
    
def steer(queuesList, angle):
    # print("#----- steering -----#")
    # print("angle =", angle)
    queuesList[SteerMotor.Queue.value].put(
    {
        "Owner": SteerMotor.Owner.value,
        "msgID": SteerMotor.msgID.value,
        "msgType": SteerMotor.msgType.value,
        "msgValue": angle
    }
    )

def brake(queuesList):
    print("#----- braking -----#")
    queuesList[Brake.Queue.value].put(
    {
        "Owner": Brake.Owner.value,
        "msgID": Brake.msgID.value,
        "msgType": Brake.msgType.value,
        "msgValue": 0
    }
    )
    
def start_recording(queuesList):
    queuesList[Record.Queue.value].put(
    {
        "Owner": Record.Owner.value,
        "msgID": Record.msgID.value,
        "msgType": Record.msgType.value,
        "msgValue": True
    }
    )
def stop_recording(queuesList):
    queuesList[Record.Queue.value].put(
    {
        "Owner": Record.Owner.value,
        "msgID": Record.msgID.value,
        "msgType": Record.msgType.value,
        "msgValue": False
    }
    )