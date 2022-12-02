# Winding algorithm https://en.wikipedia.org/wiki/Point_in_polygon , https://stackoverflow.com/questions/217578/how-can-i-determine-whether-a-2d-point-is-within-a-polygon

import numpy as np 
from collections import namedtuple
from pprint import pprint as pp
import sys
from matplotlib import pyplot as plt
import matplotlib.pyplot as plt
from matplotlib.patches import PathPatch, Polygon
from matplotlib.path import Path

def winding_number(points, *polygons):
    # TODO calculate the winding number for each point for given a set of polygons 
    return  # return whether each point inside or not 

points = [[5,5], [0,0], [-10, 5], [0, 5], [2,2], [8, 5], [10, 10]] 
polygon1 = [(9, 5),(23,-2),(-2,-2),(-2,2)]
polygon2 = [(1,0),(-1,1),(-1,-1)]

is_inside = winding_number(points, polygon1, polygon2)

polygon1 = Polygon(polygon1, alpha=0.3, color=[1, 0, 0])
polygon2 = Polygon(polygon2, alpha=1.0, color=[1, 1, 1])

fig, ax = plt.subplots(1,1)
ax.add_patch(polygon1)
ax.add_patch(polygon2)
points = np.array(points)

test_points_in = []
test_points_out = []

for is_inside, point in zip(is_inside, points):
    if(is_inside):
        test_points_in.append(point)
    else:
        test_points_out.append(point)

test_points_in = np.array(test_points_in)
test_points_out = np.array(test_points_out)
plt.scatter(test_points_in[:,0], test_points_in[:,1])
plt.scatter(test_points_out[:,0], test_points_out[:,1])
plt.show()






