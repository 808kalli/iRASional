import time
from src.move.threads.movements.basic import setSpeed, steer, brake
from src.move.threads.movements.parking import draw_parking_trajectory

def parking_reaction(queuesList, offset):
    # parking
    a, d = draw_parking_trajectory(0, offset)
    i = 0
    angle = 0
    setSpeed(queuesList, -15)
    for dist in d:
        t = dist / 15.0
        if a[i] <= 0 and angle > 0:
            angle = -5
        angle = angle + a[i]
        if angle > 23:
            angle = 23
        elif angle < -23:
            angle = -23
        steer(queuesList, angle)
        i = i + 1
        time.sleep(t)

    steer(queuesList, 23)
    setSpeed(queuesList, 15)
    time.sleep(1)
    brake(queuesList)
    steer(queuesList, 0)
    time.sleep(5)
    
    #unparking
    
    a, d = draw_parking_trajectory(0, 0, 0)
    i = 0
    angle = 0
    setSpeed(queuesList, -15)
    time.sleep(1)
    setSpeed(queuesList, 15)
    for dist in d:
        t = dist / 15.0
        if a[i] <= 0 and angle > 0:
            angle = -5
        angle = angle + a[i]
        if angle > 23:
            angle = 23
        elif angle < -23:
            angle = -23
        steer(queuesList, -angle)
        i = i + 1
        time.sleep(t)

    steer(queuesList, 23)
    setSpeed(queuesList, 15)
    time.sleep(0.8)
    
