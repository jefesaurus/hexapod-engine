import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt
from leg import * 
from math import pi

mpl.rcParams['legend.fontsize'] = 10
fig = plt.figure()
ax = fig.gca(projection='3d')


origin = np.array([0,0,0])
alpha = [0, pi/2, 0, 0]
radius = [1., .5, 1.5, 2]
displacement = [-.5,0,0,0]

theta = [[0, 0, pi/4, -pi/2],
        [pi/3, 0, pi/4, -pi/2],
        [2*pi/3, 0, pi/4, -pi/2],
        [pi, 0, pi/4, -pi/2],
        [4*pi/3, 0, pi/4, -pi/2],
        [5*pi/3, 0, pi/4, -pi/2]]

legs = [None]*6

for i in xrange(len(theta)):
  legs[i] = leg(theta[i], alpha, radius, displacement)

X = []
Y = []
Z = []


def plotLeg(ax, points):
  X = [point[0] for point in points]
  Y = [point[1] for point in points]
  Z = [point[2] for point in points]

  for i in range(len(points)-1):
    ax.plot([X[i],X[i+1]],[Y[i],Y[i+1]],[Z[i],Z[i+1]])

  return (X, Y, Z)

def plotBoundingBox(ax, X, Y, Z):
  max_range = max([max(X)-min(X), max(Y)-min(Y), max(Z)-min(Z)])
  Xb = 0.5*max_range*np.mgrid[-1:2:2,-1:2:2,-1:2:2][0].flatten() + 0.5*(max(X)+min(X))
  Yb = 0.5*max_range*np.mgrid[-1:2:2,-1:2:2,-1:2:2][1].flatten() + 0.5*(max(Y)+min(Y))
  Zb = 0.5*max_range*np.mgrid[-1:2:2,-1:2:2,-1:2:2][2].flatten() + 0.5*(max(Z)+min(Z))
  # Comment or uncomment following both lines to test the fake bounding box:
  for xb, yb, zb in zip(Xb, Yb, Zb):
    ax.plot([xb], [yb], [zb], 'w')

for i in xrange(len(legs)):
  points = (origin,) + legs[i].getAllJoints(theta[i])
  (newX,newY,newZ) = plotLeg(ax, points)
  X.extend(newX)
  Y.extend(newY)
  Z.extend(newZ)


plotBoundingBox(ax, X, Y, Z)
ax.legend()
plt.show()

