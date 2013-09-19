import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from math import pi
from chassis import *
from random import random

mpl.rcParams['legend.fontsize'] = 10
fig = plt.figure()
ax = fig.gca(projection='3d')

c = chassis()

#target = [[random()*3-1.5,random()*3-1.5,random()-3] for i in range(6)]
target = [[random()*3-1.5,random()*3-1.5,random()-3]]*6
angles = [c.getAngles(target[i], i) for i in range(6)]
c.theta = angles
segments = c.getChassisSegments()

X = []
Y = []
Z = []

for seg in segments:
  X.append(seg[1][0])
  Y.append(seg[1][1])
  Z.append(seg[1][2])

def plotSegments(ax, segments):
  for seg in segments:
    ax.plot([seg[0][0], seg[1][0]],[seg[0][1],seg[1][1]],[seg[0][2],seg[1][2]])

def plotBoundingBox(ax, X, Y, Z):
  max_range = max([max(X)-min(X), max(Y)-min(Y), max(Z)-min(Z)])
  Xb = 0.5*max_range*np.mgrid[-1:2:2,-1:2:2,-1:2:2][0].flatten() + 0.5*(max(X)+min(X))
  Yb = 0.5*max_range*np.mgrid[-1:2:2,-1:2:2,-1:2:2][1].flatten() + 0.5*(max(Y)+min(Y))
  Zb = 0.5*max_range*np.mgrid[-1:2:2,-1:2:2,-1:2:2][2].flatten() + 0.5*(max(Z)+min(Z))
  # Comment or uncomment following both lines to test the fake bounding box:
  for xb, yb, zb in zip(Xb, Yb, Zb):
    ax.plot([xb], [yb], [zb], 'w')

#def update_lines(num, line):
#  segments = c.getChassisSegments()
  

plotSegments(ax, segments)
plotBoundingBox(ax, X, Y, Z)

#ax.legend()

#line_ani = animation.FuncAnimation(fig, update_lines, 25, fargs=(data, lines), interval=50, blit=False)

plt.show()
