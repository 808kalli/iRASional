import threading
import time
import base64
import logging
from collections import deque

from multiprocessing import Pipe
from src.utils.messages.allMessages import (
ImuData, 
serialCamera,
EngineRun,
TrafficSign,
BatteryLvl,
Pedestrian,
Path,
Calculate,
Estimate,
InterDistance,
MoveConfig
)
from src.templates.threadwithstop import ThreadWithStop

from src.move.threads.movements.basic import setSpeed, steer, brake, start_recording, stop_recording
import src.move.threads.movements.lane_following as lf
import src.move.threads.movements.lane_following_old as lf_old
# def roundabout_navigation(outPs, imu, inPs, queue, flag): #direction
#     start_yaw = imu.get_yaw()
#     theta = start_yaw%90
#     if (theta >45):
#         theta = theta - 90
#     setSpeed(outPs, vel)
#     time.sleep(1)
#     steer(outPs, 21.5)
#     if (start_yaw > 315):
#       start_yaw = start_yaw-360
#     while(imu.get_yaw() - start_yaw < 90 + theta):
#       continue
    
#     lanefollowfor(inPs,outPs, 10,queue, flag)
        
#     start_yaw = imu.get_yaw()
#     theta = start_yaw%90
#     if (theta >45):
#         theta = theta - 90
#     setSpeed(outPs, vel)
#     time.sleep(1)
#     steer(outPs, 21.5)
#     if (start_yaw > 315):
#       start_yaw = start_yaw-360
#     while(imu.get_yaw() - start_yaw < 90 + theta):
#       continue

def roundabout_small(queuesList):
    setSpeed(queuesList,20)
    time.sleep(1.5)

    steer(queuesList,22)
    time.sleep(3.1)

    steer(queuesList,-18)
    #setSpeed(queuesList,20)
    time.sleep(0.8)

    steer(queuesList,22)
    time.sleep(2.9)

    steer(queuesList,0)
    time.sleep(0.5)
    brake(queuesList)

def roundabout_medium(queuesList):
    setSpeed(queuesList,20)
    time.sleep(1.5)

    steer(queuesList,22)
    time.sleep(2.7)

    steer(queuesList,-22)
    #setSpeed(queuesList,20)
    time.sleep(5.1)

    steer(queuesList,22)
    time.sleep(2.9)

    steer(queuesList,0)
    time.sleep(0.5)
    brake(queuesList)

def roundabout_looong(queuesList):
    setSpeed(queuesList,20)
    time.sleep(1.5)

    steer(queuesList,22)
    time.sleep(2.6)

    steer(queuesList,-22)
    #setSpeed(queuesList,20)
    time.sleep(11)

    steer(queuesList,22)
    time.sleep(2.9)

    steer(queuesList,0)
    time.sleep(0.1)
    brake(queuesList)    
      
def roundabout_looonger(queuesList):
    setSpeed(queuesList, 50)
    time.sleep(0.59)

    steer(queuesList, 22)
    time.sleep(1)
    
    steer(queuesList, -22)
    time.sleep(6.9)
    
    steer(queuesList, 22)
    time.sleep(1)
    
    steer(queuesList, 0)
    time.sleep(0.5)
    brake(queuesList)

def roundabout_looonger2(queuesList):
    setSpeed(queuesList, 90)
    time.sleep(0.31)

    steer(queuesList, 22)
    time.sleep(0.52)
    
    steer(queuesList, -22)
    time.sleep(3)
    
    steer(queuesList, 22)
    time.sleep(0.19)
    
    steer(queuesList, 0)
    time.sleep(0.25)
    brake(queuesList)