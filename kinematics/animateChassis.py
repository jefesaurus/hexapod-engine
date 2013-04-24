import matplotlib as mpl
import mpl_toolkits.mplot3d.axes3d as p3
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from math import pi,cos,sin
from chassis import *
from chassisParams import chassisParams
from pose import pose
from random import random

fig = plt.figure()
ax = p3.Axes3D(fig)

cp = chassisParams()
c = chassis(cp)

target = [[random()*3-1.5,random()*3-1.5,random()-3]]*6
angles = [c.getAngles(target[i], i) for i in range(len(c.legs))]
segments = c.getChassisSegments(angles)


lines = [ax.plot([dat[0][0],dat[1][0]],[dat[0][1],dat[1][1]],[dat[0][2],dat[1][2]])[0] for dat in segments]

def update_lines(num, lines):
  currentAngle = 2*pi*num/180.
  newTarget = [[3.25*sin(i*pi/3),3.25*cos(i*pi/3),-1.5] for i in xrange(6)]
  c.chassisPose = pose((0*1.25*sin(3*currentAngle),0*1.25*cos(3*currentAngle),.5*cos(.5*currentAngle)),(currentAngle,0,0))
  newThetas = [c.getAngles(newTarget[i], i) for i in range(len(c.legs))]
  newSegments = c.getChassisSegments(newThetas)
  for line,data in zip(lines,newSegments):
    line.set_data([[data[0][0],data[1][0]],[data[0][1],data[1][1]]])
    line.set_3d_properties([data[0][2],data[1][2]])



ax.set_xlim3d([-3.0, 3.0])
ax.set_ylim3d([-3.0, 3.0])
ax.set_zlim3d([-3.0, 0.0])


line_ani = animation.FuncAnimation(fig, update_lines, 2000, fargs=(lines,), interval=10, blit=False)

plt.show()
