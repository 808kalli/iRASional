import time
from src.move.threads.movements.basic import setSpeed, steer, brake, start_recording, stop_recording
from src.move.threads.movements.stop_reaction import stop_reaction
from src.move.threads.movements.parking_reaction import parallel_park, parallel_unpark 
from src.move.threads.movements.roundabout_reaction import roundabout_small, roundabout_medium, roundabout_looong, roundabout_looonger, roundabout_looonger2
from src.move.threads.movements.highway_entry_reaction import highway_entry
from src.move.threads.movements.highway_exit_reaction import highway_exit
from src.move.threads.movements.crosswalk_reaction import crosswalk_reaction


def sign_reaction(queuesList, sign):
    if sign == "Stop":
        stop_reaction(queuesList)

    elif sign == "roundabout":
        time.sleep(3)
        roundabout_looong(queuesList)

    elif sign == "Parking":
        print("parkarw")
        parallel_park(queuesList)
        time.sleep(2)
        print("parkara")
        parallel_unpark(queuesList)
        print("feugw")
    
    elif sign == "highway_entry":
        highway_entry(queuesList)
    
    elif sign == "highway_exit":
        highway_exit(queuesList)
    
    elif sign == "crosswalk":
        crosswalk_reaction(queuesList)

    

    
    
    
            