# Ray tracing algorithm https://rosettacode.org/wiki/Ray-casting_algorithm, https://github.com/bubnicbf/Ray-Casting-Algorithm

import numpy as np 
from collections import namedtuple
from pprint import pprint as pp
import sys
from matplotlib import pyplot as plt
import matplotlib.pyplot as plt
from matplotlib.patches import PathPatch
from matplotlib.path import Path


axes = plt.gca()

Pt = namedtuple('Pt', 'x, y')               # Point
Edge = namedtuple('Edge', 'a, b')           # Polygon edge from a to b
Poly = namedtuple('Poly', 'name, edges')    # Polygon
 
_eps = 0.00001
_huge = sys.float_info.max
_tiny = sys.float_info.min
 
def ray_intersects_segment(p, edge):
    ''' takes a point p and an edge of two endpoints a,b=Pt() of a line segment returns boolean
    '''
    # TODO given point and a ray, decide point would intersect then the ray
 
def _odd(x): return x%2 == 1
 
def ispointinside(p, poly):
    return _odd(sum(ray_intersects_segment(p, edge) for edge in poly.edges ))

obs = [(9, 5),(23,-2),(-2,-2),(-2,2),(-6, 9),(1,0),(-1,1),(-1,-1),(-6,4)]
polys = [
    Poly(name='obs', edges=(
    Edge(a=Pt(x=obs[0][0], y=obs[0][1]), b=Pt(x=obs[1][0], y=obs[1][1])),
    Edge(a=Pt(x=obs[1][0], y=obs[1][1]), b=Pt(x=obs[2][0], y=obs[2][1])),
    Edge(a=Pt(x=obs[2][0], y=obs[2][1]), b=Pt(x=obs[3][0], y=obs[3][1])),
    Edge(a=Pt(x=obs[3][0], y=obs[3][1]), b=Pt(x=obs[4][0], y=obs[4][1])),
    Edge(a=Pt(x=obs[4][0], y=obs[4][1]), b=Pt(x=obs[5][0], y=obs[5][1])),
    Edge(a=Pt(x=obs[5][0], y=obs[5][1]), b=Pt(x=obs[6][0], y=obs[6][1])),
    Edge(a=Pt(x=obs[6][0], y=obs[6][1]), b=Pt(x=obs[7][0], y=obs[7][1])),
    Edge(a=Pt(x=obs[7][0], y=obs[7][1]), b=Pt(x=obs[8][0], y=obs[8][1])),
    Edge(a=Pt(x=obs[8][0], y=obs[8][1]), b=Pt(x=obs[0][0], y=obs[0][1])),
    ))]

path = Path( obs, [Path.MOVETO,Path.LINETO,Path.LINETO,Path.LINETO,Path.CLOSEPOLY,Path.MOVETO,Path.LINETO,Path.LINETO,Path.CLOSEPOLY])
patch = PathPatch(path, alpha = 0.3)
axes.add_patch(patch)

testpoints = (Pt(x=5, y=5), Pt(x=0, y=0), Pt(x=-10, y=5), Pt(x=0, y=5), Pt(x=2, y=2), Pt(x=8, y=5), Pt(x=10, y=10))
test_points_in = []
test_points_out = []
for point in testpoints:
    for poly in polys:
        is_inside = ispointinside(point, poly)
        if(is_inside):
            test_points_in.append((point.x, point.y))
        else:
            test_points_out.append((point.x, point.y))

test_points_in = np.array(test_points_in)
test_points_out = np.array(test_points_out)
plt.scatter(test_points_in[:,0], test_points_in[:,1])
plt.scatter(test_points_out[:,0], test_points_out[:,1])
plt.show()





