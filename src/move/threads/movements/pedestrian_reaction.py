import time
from src.move.threads.movements.basic import setSpeed, steer, brake

def pedestrian_reaction(queuesList):
    print("seen pedestrian")
    brake(queuesList)
    time.sleep(2)