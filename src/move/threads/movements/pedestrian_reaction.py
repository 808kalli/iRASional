import time
from src.move.threads.movements.basic import setSpeed, steer, brake



def pedestrian_reaction(queuesList,ped_seen):
    if ped_seen==True:
        brake(queuesList)
        time.sleep(3)
        
