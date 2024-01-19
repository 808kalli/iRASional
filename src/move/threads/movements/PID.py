import time
import cv2   as cv
import numpy as np
import math

class PID:
    def __init__(self, pidVals, targetVal, axis=0, limit=None):
        self.pidVals = pidVals
        self.targetVal = targetVal
        self.axis = axis
        self.pError = 0
        self.limit = limit
        self.I = 0
        self.pTime = 0

    def update(self, cVal):
        # Current Value - Target Value
        t = time.time() - self.pTime
        error = cVal #- self.targetVal
        P = self.pidVals[0] * error
        self.I = self.I + (self.pidVals[1] * error * t)
        D = (self.pidVals[2] * (error - self.pError)) / t

        result = P + self.I + D + self.targetVal

        if self.limit is not None: #! if we have limitsremove if
            result = float(np.clip(result, self.limit[0], self.limit[1]))
        self.pError = error
        self.ptime = time.time()

        return result