import time
from src.move.threads.movements.basic import setSpeed, steer, brake
from src.move.threads.movements.PID import PID
<<<<<<< HEAD
import numpy as np
import math
import matplotlib.pyplot as plt
from scipy.spatial import distance

xs = (1.035, 0)        #starting point
xf = (0, 1.035)        #edning point
phi = 89.9999      #starting angle (deg)


def find_intersection(m1, b1, m2, b2):
    # Calculate the x-coordinate of the intersection point
    x_intersection = (b2 - b1) / (m1 - m2)

    # Calculate the y-coordinate using either line equation
    y_intersection = m1 * x_intersection + b1

    return x_intersection, y_intersection


def calculate_incircle(vertex1, vertex2, vertex3, xs, xf, phi):

    tangent_points = []

    side_a = distance.euclidean(vertex2, vertex3)
    side_b = distance.euclidean(vertex1, vertex3)
    side_c = distance.euclidean(vertex1, vertex2)

    # Calculate incircle radius
    # Use Heron's formula to find the area of the triangle
    s = 0.5 * (side_a + side_b + side_c)
    triangle_area = (s * (s - side_a) * (s - side_b) * (s - side_c)) ** 0.5

    # Find the radius of the incircle
    radius = triangle_area/s

    # Calculate the angles of the triangle using the law of cosines
    angle_a = np.arccos((side_b**2 + side_c**2 - side_a**2) / (2 * side_b * side_c))
    angle_b = np.arccos((side_c**2 + side_a**2 - side_b**2) / (2 * side_c * side_a))
    angle_c = np.pi - angle_a - angle_b

    # Calculate the incenter using the angle bisector theorem
    incenter_x = (side_a * vertex1[0] + side_b * vertex2[0] + side_c * vertex3[0]) / (side_a + side_b + side_c)
    incenter_y = (side_a * vertex1[1] + side_b * vertex2[1] + side_c * vertex3[1]) / (side_a + side_b + side_c)

    tangent_lenth = s - side_c

    tangent_point_1_x = vertex3[0]-math.cos(np.radians(phi))*tangent_lenth
    tangent_point_1_y = vertex3[1]-math.sin(np.radians(phi))*tangent_lenth
    tangent_points.append((tangent_point_1_x, tangent_point_1_y))

    tangent_point_2_x = vertex3[0]-tangent_lenth if xf[0] < xs[0] else vertex3[0]+tangent_lenth
    tangent_point_2_y = vertex3[1]
    tangent_points.append((tangent_point_2_x, tangent_point_2_y))

    return radius, incenter_x, incenter_y, tangent_points


def find_targ(angle, direction):
    offset = angle%90
    if offset > 45:
        offset = 90-offset
    if (direction == "RIGHT"):
        target = angle + 90 - offset
    if (direction == "LEFT"):
        target = angle - 90 + offset
    if target < 0:
        target = target + 360
    if target > 360:
=======
from scipy.interpolate import CubicSpline
import numpy as np

from src.utils.messages.allMessages import (
    CurrentSpeed
)

def find_targ(angle, direction):
    print("angle ", angle)
    offset = angle%90
    if offset > 45:
        offset = offset - 90
    if (direction == "RIGHT"):
        target = angle + 90 - offset
    elif (direction == "LEFT"):
        target = angle - 90 - offset
    elif (direction == "STRAIGHT"):
        target = angle - offset
    if target < 0:
        target = target + 360
    if target >=360:
>>>>>>> 34965e969f6bceba984ba243fb9b98c0d90b4010
        target = target - 360
    print("target= ", target)
    return target

<<<<<<< HEAD
        

def right_turn(pipe, queuesList, dist1, angle, dist2, speed):
    t1 = dist1/float(speed)
    while not pipe.poll():
        continue        
    target = float(pipe.recv()["value"]["yaw"])
    pipe.send("ready")
    gostraight(pipe, queuesList, target, t1)
    
    curr = target
    target = find_targ(curr, "RIGHT")
    steer(queuesList, angle)
    while (target > curr or curr > 315):
        if pipe.poll():
            curr = float(pipe.recv()["value"]["yaw"])
            pipe.send("ready")
            print("current= ", curr)
        if (target == 360 and curr < 45):
            break
    steer(queuesList, 0)
    t2 = dist2/float(speed)
    while not pipe.poll():
        continue    
    target = float(pipe.recv()["value"]["yaw"])
    pipe.send("ready")
    gostraight(pipe, queuesList, target, t2)
    
def left_turn(pipe, queuesList, dist1, angle, dist2, speed):
    t1 = dist1/float(speed)
    while not pipe.poll():
        continue
    target = float(pipe.recv()["value"]["yaw"])
    pipe.send("ready")
    gostraight(pipe, queuesList, target, t1)
    
    curr = target
    target = find_targ(curr, "LEFT")
    steer(queuesList, angle)
    while (target < curr or curr < 45):
        if pipe.poll():
            curr = float(pipe.recv()["value"]["yaw"])
            pipe.send("ready")
            print("current= ", curr)
        if (target == 0 and curr > 315):
            break
    
    steer(queuesList, 0)
    t2 = dist2/float(speed)
    while not pipe.poll():
        continue
    target = float(pipe.recv()["value"]["yaw"])
    pipe.send("ready")
    gostraight(pipe, queuesList, target, t2)

def gostraight(pipe, queuesList, target, t):
    start_time = time.time()
    while (time.time() - start_time <= t):
        if pipe.poll():
            data = pipe.recv()["value"]
            pipe.send("ready")
            pid = PID((0.002, 0.0, 0.0001), target, [-15, 15])
            angle = pid.update(float(data["yaw"]))
            steer(queuesList, angle)

def intersection_navigation(xs, xf, phi):
    # Line 1
    m1 = math.tan(np.radians(phi))
    b1 = -xs[0]*math.tan(np.pi + np.radians(phi))

    # Line 2
    m2 = 0  # Change this to the desired slope
    b2 = xf[1]  # Change this to the desired y-intercept

    # Find intersecting point of two lines
    intersection_x, intersection_y = find_intersection(m1, b1 ,m2, b2)

    R_com, incenter_x, incenter_y, tangent_points = calculate_incircle((xs[0]-2*xf[1]/m1, xs[1]-2*xf[1]), (xf[0]-2*xs[0], xf[1]), (intersection_x, intersection_y), xs, xf, phi)

    trajectory_points = [xs, tangent_points[0], tangent_points[1], xf]
    print("Trajectory points: ", trajectory_points)

    #find trajectory
    theta = np.pi/2 - np.radians(phi)
    straignt1 = distance.euclidean(trajectory_points[0], trajectory_points[1])
    straight2 = distance.euclidean(trajectory_points[2], trajectory_points[3])
    L = 0.26
    lf = 0.15
    R = L/math.sin(theta)
    steer_angle = math.asin(math.sqrt(L**2/(R_com**2+2*lf*L-lf**2)))      #steer angle
    steer_angle = steer_angle if xf[0] > xs[0] else -steer_angle            #steer_angle > 0 for right turn, steer_angle < 0 for left turn

    print(straignt1)
    print(np.degrees(steer_angle))
    print(straight2)

    # # #==========================PLOT==========================

    # # Plotting the points
    # plt.plot(xs[0], xs[1], 'go')
    # plt.plot(xf[0], xf[1], 'go')
    # plt.plot(xs[0]-2*xf[1]/m1, xs[1]-2*xf[1], 'go')
    # plt.plot(xf[0]-2*xs[0], xf[1], 'go')

    # # Plot the incenter
    # plt.plot(incenter_x, incenter_y, 'bo', label='Incenter')

    # tangent_x, tangent_y = zip(*tangent_points)
    # plt.plot(tangent_x, tangent_y, 'go', label='Tangent Points')

    # # Generate x values
    # x_values = np.linspace(-30, 30, 1000)

    # # Calculate corresponding y values for both lines
    # y_values_line1 = m1*x_values + b1
    # y_values_line2 = m2*x_values + b2

    # # Plot the lines
    # plt.plot(x_values, y_values_line1)
    # plt.plot(x_values, y_values_line2)

    # # Plot the intersection point
    # plt.scatter(intersection_x, intersection_y, color='red', label='Intersection Point')

    # # draw the triangle
    # plt.plot([xs[0], xf[0], intersection_x, xs[0]],
    #          [xs[1], xf[1], intersection_y, xs[1]], 'b-')

    # # Plot the incircle
    # theta = np.linspace(0, 2*np.pi, 100)
    # incircle_x = incenter_x + R_com * np.cos(theta)
    # incircle_y = incenter_y + R_com * np.sin(theta)
    # plt.plot(incircle_x, incircle_y, 'r-', label='Incircle')


    # # Set labels and title
    # plt.xlabel('x-axis')
    # plt.ylabel('y-axis')
    # plt.title('Intersection of Two Lines')
    # plt.axhline(0, color='black', linewidth=0.5)
    # plt.axvline(0, color='black', linewidth=0.5)
    # plt.grid(color='gray', linestyle='--', linewidth=0.5)
    # plt.axis('equal')
    # plt.legend()

    # # Show the plot
    # plt.show()

    return straignt1, np.degrees(steer_angle), straight2

=======
def gostraight(pipe, queuesList, t):
    dummy = pipe.recv()
    pipe.send("ready")
    while (not pipe.poll()):
        continue
    target = find_targ(float(pipe.recv()["value"]["yaw"]), "STRAIGHT")
    pipe.send("ready")
    start_time = time.time()
    pid = PID((4, 0.0, 0.05), target, [-15, 15])
    while (time.time() - start_time <= t):
        if pipe.poll():
            data = pipe.recv()["value"]
            if (float(data["yaw"]) > 315):
                angle = float(data["yaw"]) - 360
            else:
                angle = float(data["yaw"])
            print("angle", angle)
            pipe.send("ready")
            angle = pid.update(angle)
            print("steer", angle)
            steer(queuesList, angle)

def draw_trajectory(xs, xf, phi):
    offset = 0
    x = [0, 20 - offset, 72 - offset]
    y = [0, 15 - offset, 25 - offset]
    xx = np.linspace(min(x), max(x), 3)
    yy = CubicSpline(x, y)
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
    for i in range(len(angles)):
        angles[i] = 90 - angles[i]

    return np.diff(angles), distances

def draw_trajectory_left(xs, xf, phi):
    offset = 0
    x = [0, 20 - offset, 68 - offset]
    y = [0, 50 - offset, 100 - offset]
    xx = np.linspace(min(x), max(x), 8)
    yy = CubicSpline(x, y)
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
    for i in range(len(angles)):
        angles[i] = 90 - angles[i]

    return (-1) * np.diff(angles), distances

def intersection_navigation(current, pipe, queuesList, offset = 0, speed=15):
    if current == "RIGHT":
        gostraight(pipe, queuesList, 0.5)
        angles, distances = draw_trajectory(offset, 0, 0)
        i = 0
        angle=0
        for dist in distances:
            t = dist / 15.0
            angle = angle + angles[i]
            steer(queuesList, angle)
            i = i + 1
            time.sleep(t)
    elif current == "LEFT":
        gostraight(pipe, queuesList, 0.5)
        angles, distances = draw_trajectory_left(offset, 0, 0)
        i = 0
        angle=0
        for dist in distances:
            t = dist / 15.0
            angle = angle + angles[i]
            steer(queuesList, angle)
            i = i + 1
            time.sleep(t)
    else:
        gostraight(pipe, queuesList, int(140/int(speed)))
<<<<<<< HEAD
>>>>>>> 34965e969f6bceba984ba243fb9b98c0d90b4010
=======
>>>>>>> 72fa1521fdb8afbbaa91a55b972aacaf77f43c84
