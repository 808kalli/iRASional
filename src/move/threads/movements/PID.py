import time
import numpy as np

class PID:
    def __init__(self, pidVals, targetVal, limits):
        
        self.pidVals = pidVals
        self.Kp = self.pidVals[0]
        self.Ki = self.pidVals[1] 
        self.Kd = self.pidVals[2]
         
        self.targetVal = targetVal
        self.pError = 0
        self.limits = limits
        self.I = 0
        self.D = 0
        self.K = 0
        self.pTime = time.time()

    def update(self, cVal):
        t = time.time() - self.pTime
        error = self.targetVal - cVal
        self.P = self.Kp * error
        self.I = self.I + (self.Ki * error * t)
        self.D = (self.Kd * (error - self.pError)) / t
<<<<<<< HEAD

        result = self.P + self.I + self.D
=======
        result = self.P + self.I + self.D
        
>>>>>>> 34965e969f6bceba984ba243fb9b98c0d90b4010

        if self.limits is not None:
            result = float(np.clip(result, self.limits[0], self.limits[1]))
        result = round(result)
        self.pError = error
        self.ptime = time.time()
<<<<<<< HEAD

=======
>>>>>>> 34965e969f6bceba984ba243fb9b98c0d90b4010
        return result