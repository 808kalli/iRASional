import time
from src.move.threads.movements.basic import setSpeed, steer, brake


def overtake(queuesList): #inPs,,queue,capture
    # steer(queuesList,-22)
    # setSpeed(queuesList,15)
    # time.sleep(3)
    # steer(queuesList,15)
    # lanefollowfor(inPs,queuesList,6,queue,capture)
    # steer(queuesList,22)
    # setSpeed(queuesList,15)
    # time.sleep(3)
    # steer(queuesList,-15)

    steer(queuesList,-22)
    setSpeed(queuesList,20)
    time.sleep(2.1)
    steer(queuesList,19)
    time.sleep(2)
    steer(queuesList,22)
    #setSpeed(queuesList,20)
    time.sleep(2.85)
    #setSpeed(queuesList,0)
    #lanefollowfor(inPs,queuesList,6,queue,capture)
    #brake(queuesList)
    # time.sleep(5)

    steer(queuesList,-20)
    setSpeed(queuesList,20)
    time.sleep(2)
    brake(queuesList)