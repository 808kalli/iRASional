import numpy as np
from numpy.linalg import inv

def predict(x, F, G, accel, P, Q):
    x = F@x + G@accel
    P = F@P@F.T + Q

    return x, P


def update(x, z, H, P, R):
    y = z - H@x
    S = H@P@H.T + R
    K = P@H.T@inv(S)
    x = x + K@y
    P = (np.eye(4, 4) - K@H)@P

    return x, P

