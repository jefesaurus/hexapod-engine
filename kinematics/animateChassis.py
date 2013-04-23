import matplotlib as mpl
import mpl_toolkits.mplot3d.axes3d as p3
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from math import pi,cos,sin
from chassis import *
from random import random

fig = plt.figure()
ax = p3.Axes3D(fig)

c = chassis()

#target = [[random()*3-1.5,random()*3-1.5,random()-3] for i in range(6)]
target = [[random()*3-1.5,random()*3-1.5,random()-3]]*6
angles = [c.getAngles(target[i], i) for i in range(6)]
c.theta = angles
segments = c.getChassisSegments()


lines = [ax.plot([dat[0][0],dat[1][0]],[dat[0][1],dat[1][1]],[dat[0][2],dat[1][2]])[0] for dat in segments]

def update_lines(num, lines):
  currentAngle = 2*pi*(num%180/180.)
  newTarget = [[3.5*cos(i*pi/3),3.5*sin(i*pi/3),-1.5] for i in xrange(6)]
  c.updateAttitude([cos(currentAngle),sin(currentAngle),0],0,0,0)
  c.theta = [c.getAngles(newTarget[i], i) for i in range(6)]
  newSegments = c.getChassisSegments()
  for line,data in zip(lines,newSegments):
    line.set_data([[data[0][0],data[1][0]],[data[0][1],data[1][1]]])
    line.set_3d_properties([data[0][2],data[1][2]])



ax.set_xlim3d([-2.0, 2.0])
ax.set_ylim3d([-2.0, 2.0])
ax.set_zlim3d([-3.0, 0.0])


line_ani = animation.FuncAnimation(fig, update_lines, 180, fargs=(lines,), interval=5, blit=False)

plt.show()
