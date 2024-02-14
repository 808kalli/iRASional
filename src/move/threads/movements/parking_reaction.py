
import time
from src.move.threads.movements.basic import setSpeed, steer, brake


def parallel_park(queuesList):
    steer(queuesList,-21)
    setSpeed(queuesList,15)
    time.sleep(2.5)
    #record(inPs,1)
    vel = -15
    steer(queuesList,21)
    setSpeed(queuesList,vel)
    time.sleep(2)
    #record(inPs,3.5)
    steer(queuesList,-21)
    setSpeed(queuesList,vel)
    time.sleep(2.8)
    #record(inPs,2.8)
    steer(queuesList,21)
    setSpeed(queuesList,15)
    time.sleep(2.2)
    #record(inPs,2)
    steer(queuesList,0)
    brake(queuesList)

def parallel_unpark(queuesList):
    steer(queuesList,21)
    setSpeed(queuesList,-15)
    time.sleep(1)
    #record(inPs,2)
    steer(queuesList,-21)
    setSpeed(queuesList,15)
    time.sleep(3)
    #record(inPs,2.2)
    steer(queuesList,20)
    setSpeed(queuesList,15)
    time.sleep(2)
    #record(inPs,4.5)
    steer(queuesList,0)
    brake(queuesList)

'''
def parparking_reaction(inPs,queuesList,queue, flag):
    offset = 0
    steer(queuesList,offset)
    setSpeed(queuesList,15)
    start_time = time.time()
    flag=False
    #time.sleep(3.4)
    #self.lanefollowfor(inPs,queuesList,3.2,queue,capture)
    for i in range(10):
        #time.sleep(0.4) 8 FORES
        lanefollowfor(inPs,queuesList,0.4,queue, flag)
        currdist=ult_distance("side")
        print(currdist)
        if(currdist<40):
            print("first position occupied")
            flag=True
    lanefollowfor(inPs,queuesList,1,queue, flag)
    
    if(flag==True):
        #time.sleep(4.8)
        lanefollowfor(inPs,queuesList,4.8,queue, flag)
    parallel_park(inPs,queuesList)
    time.sleep(2)
    #record(inPs,2)
    parallel_unpark(inPs,queuesList)
    lanefollowfor(inPs, queuesList, 5, queue, flag)
'''