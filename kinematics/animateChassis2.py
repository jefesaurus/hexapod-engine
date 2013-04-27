import matplotlib as mpl
import mpl_toolkits.mplot3d.axes3d as p3
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from math import pi,cos,sin,sqrt
from random import random

from model import *

fig = plt.figure()
ax = p3.Axes3D(fig)

chassis = animatedChassis()
initialSegments = chassis.step()
lines = [ax.plot([dat[0][0],dat[1][0]],[dat[0][1],dat[1][1]],[dat[0][2],dat[1][2]])[0] for dat in initialSegments]

def update_lines(num, lines, chassis):
  #If we have reached the end of this step
  #Get FK for drawing
  newSegments = chassis.step()
  for line,data in zip(lines,newSegments):
    line.set_data([[data[0][0],data[1][0]],[data[0][1],data[1][1]]])
    line.set_3d_properties([data[0][2],data[1][2]])


ax.set_xlim3d([-3.0, 3.0])
ax.set_ylim3d([-3.0, 3.0])
ax.set_zlim3d([-3.0, 0.0])


line_ani = animation.FuncAnimation(fig, update_lines, 2000, fargs=(lines,chassis), interval=10, blit=False)

plt.show()
