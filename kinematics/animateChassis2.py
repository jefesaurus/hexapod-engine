import matplotlib as mpl
import mpl_toolkits.mplot3d.axes3d as p3
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from math import pi,cos,sin,sqrt
from random import random

from modelNew import *

fig = plt.figure()
ax = p3.Axes3D(fig)

chassis = animatedChassis()
chassis.updateVelocity((.01,.00),(.00))
initialSegments = chassis.step()
lines = [ax.plot([dat[0][0],dat[1][0]],[dat[0][1],dat[1][1]],[dat[0][2],dat[1][2]])[0] for dat in initialSegments]
speed = .05

def update_lines(num, lines, chassis):
   
  chassis.updateVelocity((speed*cos(num/30.), 0*speed*sin(num/30.)),.01)

  newSegments = chassis.step()
  for line,data in zip(lines,newSegments):
    line.set_data([[data[0][0],data[1][0]],[data[0][1],data[1][1]]])
    line.set_3d_properties([data[0][2],data[1][2]])


ax.set_xlim3d([-3.0, 3.0])
ax.set_ylim3d([-3.0, 3.0])
ax.set_zlim3d([-3.0, 0.0])


line_ani = animation.FuncAnimation(fig, update_lines, 2000, fargs=(lines,chassis), interval=10, blit=False)

plt.show()
