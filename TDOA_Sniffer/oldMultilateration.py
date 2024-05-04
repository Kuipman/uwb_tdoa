# tdoa3Multilateration.py
# inspired by lo-calho.st/posts/tdoa-multilateration/

# Three observers: latitude, longitude, TDoA timestamp from first observation in seconds
measurements = [
    [34.888, -103.826, 0.0],     # first observation sets the origin timestamp for remaining observations
    [34.931, -103.805, 0.6],
    [34.921, -103.781, 2.5]
]

import pymap3d as pm

# ENU (East-North-Up)
# Converts the geodetic coordinates to "something we can work with more easily" i.e. ENU
lat0, lon0, t0 = measurements[0]
for i in range(len(measurements)):
    lat, lon, t = measurements[i]
    e, n, _ = pm.geodetic2enu(lat, lon, 0, lat0, lon0, 0)
    measurements[i] = [e, n, t]

print(measurements)   # each entry gives the distance east, north from the first observer (in meters) and its time of observation relative to the first observation

# For this exercise we're working with the speed of sound
speed = 343.   # speed of sound in meters per second

# Now, we'll need to define and solve the system of hyperbolic equations
# Use scipy's least-squares solver

import numpy as np

def functions(x0, y0, x1, y1, x2, y2, d01, d02, d12):
    """
    Given observers at (x0, y0), (x1, y1), (x2, y2) and TDOA between observers d01, d02, d12, this closure
        returns a function that evaluates the system of three hyperbolae for given event x, y.
    """

    def fn(args):
        x, y = args
        a = np.sqrt(np.power(x - x1, 2.) + np.power(y - y1, 2.)) - np.sqrt(np.power(x - x0, 2.) + np.power(y - y0, 2.)) - d01
        b = np.sqrt(np.power(x - x2, 2.) + np.power(y - y2, 2.)) - np.sqrt(np.power(x - x0, 2.) + np.power(y - y0, 2.)) - d02
        c = np.sqrt(np.power(x - x2, 2.) + np.power(y - y2, 2.)) - np.sqrt(np.power(x - x1, 2.) + np.power(y - y1, 2.)) - d12
        return [a, b, c]
    return fn

# We also specify the Jacobian matrix of the system -- a collection of
# partial derivatives of each function with respect to each independent variable

def jacobian(x0, y0, x1, y1, x2, y2, d01, d02, d12):
    def fn(args):
        x, y = args
        adx = (x - x1) / np.sqrt(np.power(x - x1, 2.) + np.power(y - y1, 2.)) - (x - x0) / np.sqrt(np.power(x - x0, 2.) + np.power(y - y0, 2.))
        bdx = (x - x2) / np.sqrt(np.power(x - x2, 2.) + np.power(y - y2, 2.)) - (x - x0) / np.sqrt(np.power(x - x0, 2.) + np.power(y - y0, 2.))
        cdx = (x - x2) / np.sqrt(np.power(x - x2, 2.) + np.power(y - y2, 2.)) - (x - x1) / np.sqrt(np.power(x - x1, 2.) + np.power(y - y1, 2.))
        ady = (y - y1) / np.sqrt(np.power(x - x1, 2.) + np.power(y - y1, 2.)) - (y - y0) / np.sqrt(np.power(x - x0, 2.) + np.power(y - y0, 2.))
        bdy = (y - y2) / np.sqrt(np.power(x - x2, 2.) + np.power(y - y2, 2.)) - (y - y0) / np.sqrt(np.power(x - x0, 2.) + np.power(y - y0, 2.))
        cdy = (y - y2) / np.sqrt(np.power(x - x2, 2.) + np.power(y - y2, 2.)) - (y - y1) / np.sqrt(np.power(x - x1, 2.) + np.power(y - y1, 2.))

        return [
            [adx, ady],
            [bdx, bdy],
            [cdx, cdy]
        ]
    return fn

# We can perform an initial guess for the location of the tag (average of the three observer locations)
# This prevents issues with points for which the function is not well-defined

xp = np.mean([x for x,y,t in measurements])
yp = np.mean([y for x,y,t in measurements])



# Finally, run the solver:
import scipy.optimize as opt

x0, y0, t0 = measurements[0]
x1, y1, t1 = measurements[1]
x2, y2, t2 = measurements[2]

F = functions(x0, y0, x1, y1, x2, y2, (t1 - t0) * speed, (t2 - t0) * speed, (t2 - t1) * speed)
J = jacobian(x0, y0, x1, y1, x2, y2, (t1 - t0) * speed, (t2 - t0) * speed, (t2 - t1) * speed)

solution, _ = opt.leastsq(F, x0=[xp, yp], Dfun=J)
x, y = solution

# Convert from ENU (relative to first receiver) back to geodetic coordinates
lat, lon, _ = pm.enu2geodetic(x, y, 0, lat0, lon0, 0)   # result is xy coordinates

print(lat, lon)

# Plot this

import matplotlib.pyplot as plt

# Create reasonable x, y bounds for visualization
max_x = max(x0, x1, x2, x)
min_x = min(x0, x1, x2, x)
range_x = max_x - min_x
min_x -= range_x * .2
max_x += range_x * .2

max_y = max(y0, y1, y2, y)
min_y = min(y0, y1, y2, y)
range_y = max_y - min_y
min_y -= range_y * .2
max_y += range_y * .2

# Create a grid of input coordinates
xs = np.linspace(min_x, max_x, 100)
ys = np.linspace(min_y, max_y, 100)
xs, ys = np.meshgrid(xs, ys)

# Evaluate the system across the grid
A, B, C = F((xs, ys))

# Plot the results
plt.scatter(x0, y0, color='r')
plt.scatter(x1, y1, color='g')
plt.scatter(x2, y2, color='b')
plt.scatter(x, y, color='k')
plt.contour(xs, ys, A, [0], colors='y')
plt.contour(xs, ys, B, [0], colors='m')
plt.contour(xs, ys, C, [0], colors='c')
plt.show()