import time
from src.move.threads.movements.basic import setSpeed, steer, brake
from src.move.threads.movements.parking_reaction import parking_reaction


def sign_reaction(queuesList, sign, pipe = None):
    if sign == "Stop":
        brake(queuesList)
        time.sleep(1)
        steer(queuesList,0)
        time.sleep(2)

    elif sign == "roundabout":
        time.sleep(3)

    elif sign == "Parking":
        parking_reaction(queuesList)
    
    elif sign == "highway_entry":
        setSpeed(queuesList,20)    
    
    elif sign == "highway_exit":
        setSpeed(queuesList,10)
    
    elif sign == "Crosswalk":
        while (pipe.poll()):
            Pedestrian =pipe.recv()["value"]
            pipe.send("ready")
            print("seen pedestrian")
            brake(queuesList)
            time.sleep(2)
        
    elif sign == "Priority":
        steer(queuesList,0)

    

    
    
    
            