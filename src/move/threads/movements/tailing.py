import time
from src.move.threads.movements.basic import setSpeed, steer
from src.move.threads.movements.PID import PID
from src.move.threads.movements.lane_following import followLane
import base64
import cv2
import numpy as np

kp = 0.1 #0.03
kd = 8 #0.006
ki = 0.0

distance_to_keep = 50

def tail(fdistpipe ,queuesList,imgpipe, K, t):
    tailingPID = PID([kp,ki,kd],distance_to_keep,[-14,15])
    start_time = time.time()
    speed = 0
    steer(queuesList,0)
    while (time.time() - start_time <= t):
        if fdistpipe.poll():
            dist = fdistpipe.recv()["value"]
            print(dist)
            fdistpipe.send("ready")
            speed_correction = tailingPID.update(dist)
            speed -= speed_correction
            print("speed: " + str(speed))
            speed = np.clip(speed, 0, 25)
            setSpeed(queuesList, speed)
        
        if imgpipe.poll():
            frame = imgpipe.recv()
            image_data = base64.b64decode(frame["value"])
            img = np.frombuffer(image_data, dtype=np.uint8)
            img = cv2.imdecode(img, cv2.IMREAD_COLOR)
            data = followLane(img,K, speed)
            if data is not None:
                angle, _ = data
            steer(queuesList, angle)
            imgpipe.send("ready")
        