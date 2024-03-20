from scipy.interpolate import CubicSpline
import numpy as np
import matplotlib.pyplot as plt

def draw_parking_trajectory(x, y, phi):
    x = [15, 20, 95, 100]
    y = [5, 5, 75, 75]

    # side1_x = np.linspace(0, 35, 100)
    # side1_y = np.zeros(100)
    # side2_x = np.zeros(100)
    # for i in range(len(side2_x)):
    #     side2_x[i] = side2_x[i] + 35
    # side2_y = np.linspace(0, 77, 100)
    # side3_y = side1_y + 77

    yy = CubicSpline(x, y)
    xx = np.linspace(min(x), max(x), 5)

    # plt.plot(side1_x, side1_y, 'k-')
    # plt.plot(side2_x, side2_y, 'k-')
    # plt.plot(side1_x, side3_y, 'k-')
    # for i in range(len(x)):
        # plt.plot(x[i], y[i], 'ro')
    # plt.plot(xx, yy(xx), 'b-')
    # plt.grid()
    # plt.show()

    yy_der = yy.derivative()
    slopes = []
    distances = []
    i = 0
    for xes in xx:
        slopes.append(yy_der(xes))

    for i in range(len(xx)):
        if i + 1 < len(xx):
            point1 = np.array([xx[i], yy(xx[i])])
            point2 = np.array([xx[i+1], yy(xx[i+1])])
            distances.append(np.linalg.norm(point1 - point2))
        else:
            break

    angles_rad = np.arctan(slopes)
    angles = np.rad2deg(angles_rad)

    return np.diff(angles), distances





# a, d = draw_parking_trajectory(0, 0, 0)

# print(a)
# print(d)
