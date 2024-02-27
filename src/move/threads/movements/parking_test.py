import time
from src.move.threads.movements.basic import setSpeed, steer, brake
import matplotlib.pyplot as plt
import numpy as np
import sympy as smp
import math as ma

#--------------------------HELPING FUNCTIONS---------------------#
def slope(x1, x2, y1, y2):
  s = (y2-y1)/(x2-x1)
  return s

def distance_of_two_points(A,B):
    return ma.sqrt(ma.pow(A[0]-B[0],2)+ma.pow(A[1]-B[1],2))

def radius_of_circle(abtl,A,B): #abtl=angle_between_two_lines, A,B=the two points 
    string_length=distance_of_two_points(A,B)
    central_angle=180-abtl #in degrees the angle between the center of the circle and the two points of contact
    angle_A=(180-central_angle)/2 #in degrees, because the triangle that is formed is isosceles
    radius=(ma.sin(ma.radians(angle_A))*string_length)/ma.sin(ma.radians(central_angle))
    return radius


def cubic_interpolate(x0, x, y):

    xdiff = np.diff(x)
    dydx = np.diff(y)
    dydx /= xdiff

    n = size = len(x)

    w = np.empty(n-1, float)
    z = np.empty(n, float)

    w[0] = 0.
    z[0] = 0.
    for i in range(1, n-1):
        m = xdiff[i-1] * (2 - w[i-1]) + 2 * xdiff[i]
        w[i] = xdiff[i] / m
        z[i] = (6*(dydx[i] - dydx[i-1]) - xdiff[i-1]*z[i-1]) / m
    z[-1] = 0.

    for i in range(n-2, -1, -1):
        z[i] = z[i] - w[i]*z[i+1]

    # find index (it requires x0 is already sorted)
    index = x.searchsorted(x0)
    np.clip(index, 1, size-1, index)

    xi1, xi0 = x[index], x[index-1]
    yi1, yi0 = y[index], y[index-1]
    zi1, zi0 = z[index], z[index-1]
    hi1 = xi1 - xi0

    # calculate cubic
    f0 = zi0/(6*hi1)*(xi1-x0)**3 + \
        zi1/(6*hi1)*(x0-xi0)**3 + \
        (yi1/hi1 - zi1*hi1/6)*(x0-xi0) + \
        (yi0/hi1 - zi0*hi1/6)*(xi1-x0)
    return f0

x = [15,20,110,115]
y = [19,19,56.5,56.5]
x = np.asfarray(x)
y = np.asfarray(y)


#--------------------------MAIN FUNCTIONS---------------------#
def automated_parallel_parking(x,y):
    n=10
    #plt.scatter(x, y)
    x_new= np.linspace(min(x), max(x), 10000)
    y_new = cubic_interpolate(x_new, x, y)
    wanted_angle=[]
    slopa = np.empty(100000, float)
    points=[] #the points where each tangect touches the curve
    slopes=[]
    for i in range (0,9900,n):
        slopa[i]=slop=slope(x_new[i], x_new[i+n], y_new[i], y_new[i+n])
        slopes.append(slop)
        x_tangent=np.linspace(x_new[i]-15,x_new[i]+15,10)
        tangent=slop*(x_tangent-x_new[i])+y_new[i]
        #plt.plot(x_tangent,tangent)
        #plt.scatter(x_new[i],y_new[i])
        points.append((x_new[i],y_new[i]))
        wanted_angle.append(ma.degrees(ma.atan(slop)))
        if (i==0):
            pass
            #plt.plot(x_new,y_new)
    p=0
    tp=0
    #print(slopes)
    for i in range (0,9900,n):
        slop2=slope(x_new[i],x_new[i+n],slopa[i], slopa[i+n])
        p=p+1
        if(-0.00005<slop2<0.00005):
            #plt.scatter(x_new[i],y_new[i])
            tp=x_new[i]
            break
    # slop=slope(x[3],x[2],y[3],y[2]) #for the last point
    # slopes.append(slop)
    # x_tangent=np.linspace(x[3]-15,x[3]+15,100)
    # tangent=slop*(x_tangent-x[3])+y[3]
    # plt.plot(x_tangent,tangent)
    # print(slopes)
    #plt.show()
    return wanted_angle, points, tp


def park_the_car (x,y,queuesList): #given imu angle 
    wanted_angle=[]
    points=[]
    wanted_angle, points, tp= automated_parallel_parking(x,y)
    angle_between_two_lines=[]
    steering_angle=[]
    for i in range(0,len(wanted_angle)-1):
        angle_between_two_lines.append((360-2*abs(wanted_angle[i]-wanted_angle[i+1]))/2)
    for i in range(0,len(wanted_angle)-1):
        point_A=points[i]
        point_B=points[i+1]
        radius=radius_of_circle(angle_between_two_lines[i],point_A,point_B)
        #print(radius)
        if(tp>points[i][0]):
         steering_angle.append(-(ma.degrees(ma.atan(26/(radius+8.5))+ma.atan(26/(radius-8.5)/2))))
        else:
            steering_angle.append((ma.degrees(ma.atan(26/(radius+8.5))-ma.atan(26/(radius-8.5)/2))))
    print("here")
    for i in reversed(steering_angle):
        start=time.time()
        while(time.time()-start < 2):
            steer(queuesList, i)
            





# x = [15,20,110,115]
# y = [19,19,56.5,56.5]
# x = np.asfarray(x)
# y = np.asfarray(y)
# park_the_car(x,y,queuesList)

